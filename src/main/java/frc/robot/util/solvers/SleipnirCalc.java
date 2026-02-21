// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.*;
import static org.wpilib.math.optimization.Constraints.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;

import org.ejml.simple.SimpleMatrix;
import org.wpilib.math.autodiff.Slice;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableBlock;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.Options;

import com.pathplanner.lib.util.FlippingUtil;

/**
 * Sleipnir-based trajectory optimization solver for shooting calculations.
 *
 * <p>Finds the initial velocity, pitch, and yaw for a game piece to hit the target
 * that minimizes
 * initial velocity. Uses direct transcription of flight dynamics including air
 * resistance (drag and
 * Magnus effect).
 *
 * <p>Ported from SolverCalc.cpp (Sleipnir C++ example).
 */
public class SleipnirCalc implements SolverIO {

    // Field/game constants
    private static final double FIELD_WIDTH = FlippingUtil.fieldSizeY; // 27 ft -> m
    private static final double FIELD_LENGTH = FlippingUtil.fieldSizeX; // 54 ft -> m
    private static final double TARGET_X = FIELD_LENGTH / 2.0;
    private static final double TARGET_Y = FIELD_WIDTH / 2.0;
    private static final double TARGET_Z = Inches.of(72).in(Meter); // m
    private static final double TARGET_RADIUS = 0.61; // m
    private static final double CONE_ANGLE = Math.PI / 4.0; // rad

    // Physics constants
    private static final double G = 9.806; // m/s^2
    private static final double RHO = 1.204; // air density kg/m^3
    private static final double BALL_RADIUS = Inches.of(5.91).in(Meters) / 12; // m
    private static final double BALL_AREA = Math.PI * BALL_RADIUS * BALL_RADIUS; // m^2
    private static final double BALL_MASS = Pounds.of(0.5).in(Kilograms); // kg
    private static final double C_D = 0.5; // drag coefficient
    private static final double C_L = 0.5; // lift coefficient
    private static final double[] OMEGA = { 0.0, -2.0, 0.0 }; // angular velocity rad/s

    // Solver constants
    private static final int N = 50; // number of timesteps
    private static final double MAX_INITIAL_VELOCITY = 10.0; // m/s

    // Mechanism constants
    private static final double SHOOTER_Z_OFFSET = 1.2; // shooter height above ground in m
    private static final double FLYWHEEL_RADIUS = Inches.of(2).in(Meters); // flywheel radius in m

    @Override
    public ShootingSolution getShootingSolution(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        // Robot state from pose
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotZ = 0.0;
        double robotVx = robotVelocity.vxMetersPerSecond;
        double robotVy = robotVelocity.vyMetersPerSecond;
        double robotYaw = robotPose.getRotation().getRadians();
        // convert robot-relative velocities into field frame
        double robotVxField = robotVx * Math.cos(robotYaw) - robotVy * Math.sin(robotYaw);
        double robotVyField = robotVx * Math.sin(robotYaw) + robotVy * Math.cos(robotYaw);
        robotVx = robotVxField;
        robotVy = robotVyField;
        double robotVz = 0.0;

        // Shooter position in field frame
        double shooterX = robotX;
        double shooterY = robotY;
        double shooterZ = robotZ + SHOOTER_Z_OFFSET;
        double shooterVx = robotVx;
        double shooterVy = robotVy;
        double shooterVz = robotVz;

        try (Problem problem = new Problem()) {
            // Duration decision variable
            // Duration decision variable (Google Java style: lowerCamelCase)
            Variable totalTime = problem.decisionVariable();
            // Alias for existing references that expect 'T'
            totalTime.setValue(1.0);
            Variable dt = totalTime.div((double) N);

            // Ball state in field frame: [x, y, z, vx, vy, vz] x N
            VariableMatrix X = problem.decisionVariable(6, N);

            VariableBlock p = X.block(0, 0, 3, N);
            VariableBlock px = X.row(0);
            VariableBlock py = X.row(1);
            VariableBlock pz = X.row(2);

            VariableBlock v = X.block(3, 0, 3, N);
            VariableBlock vx = X.row(3);
            VariableBlock vy = X.row(4);
            VariableBlock vz = X.row(5);

            // v0 relative to shooter
            SimpleMatrix shooterVel = new SimpleMatrix(3, 1, true, new double[] { shooterVx, shooterVy, shooterVz });
            VariableMatrix v0WrtShooter = X.block(3, 0, 3, 1).minus(shooterVel);

            // --- Initial guesses ---

            // Position: linear interpolation between shooter and target
            for (int k = 0; k < N; k++) {
                double t = (double) k / N;
                px.get(k).setValue(lerp(shooterX, TARGET_X, t));
                py.get(k).setValue(lerp(shooterY, TARGET_Y, t));
                pz.get(k).setValue(lerp(shooterZ, TARGET_Z, t));
            }

            // Velocity: max initial velocity toward target
            double dx = TARGET_X - shooterX;
            double dy = TARGET_Y - shooterY;
            double dz = TARGET_Z - shooterZ;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double ux = dx / dist;
            double uy = dy / dist;
            double uz = dz / dist;

            SimpleMatrix velGuess = new SimpleMatrix(
                    3,
                    1,
                    true,
                    new double[] {
                        robotVx + MAX_INITIAL_VELOCITY * ux,
                        robotVy + MAX_INITIAL_VELOCITY * uy,
                        robotVz + MAX_INITIAL_VELOCITY * uz
                    });
            for (int k = 0; k < N; k++) {
                v.col(k).setValue(velGuess);
            }

            // --- Constraints ---

            // Shooter initial position
            SimpleMatrix shooterPos = new SimpleMatrix(3, 1, true, new double[] { shooterX, shooterY, shooterZ });
            problem.subjectTo(eq(p.col(0), shooterPos));

            // Require initial velocity is below max:
            // vx^2 + vy^2 + vz^2 ≤ v_max^2
            Variable vxDiff = vx.get(0).minus(robotVx);
            Variable vyDiff = vy.get(0).minus(robotVy);
            Variable vzDiff = vz.get(0).minus(robotVz);
            Variable v2Sum = Variable.pow(vxDiff, 2.0).plus(Variable.pow(vyDiff, 2.0)).plus(Variable.pow(vzDiff, 2.0));
            problem.subjectTo(le(v2Sum, MAX_INITIAL_VELOCITY * MAX_INITIAL_VELOCITY));

            // Keep-out region (cylinder with conic bowl around target)
            double x_c = TARGET_X;
            double y_c = TARGET_Y;
            double z_c = TARGET_Z - TARGET_RADIUS / Math.tan(CONE_ANGLE);
            double tanAngle = Math.tan(CONE_ANGLE);

            for (int k = 0; k < N; k++) {
                Variable x = px.get(k);
                Variable y = py.get(k);
                Variable z = pz.get(k);

                Variable x2 = Variable.pow(x.minus(x_c), 2.0);
                Variable y2 = Variable.pow(y.minus(y_c), 2.0);
                Variable z2 = Variable.pow(z.minus(z_c), 2.0);

                Variable cylinder = x2.plus(y2).minus(TARGET_RADIUS * TARGET_RADIUS);
                Variable cone = z2.times(tanAngle * tanAngle).minus(x2).minus(y2);

                // max(cylinder, cone) >= 0
                // max(a, b) = (a + b + |a - b|) / 2
                Variable maxVal = cylinder.plus(cone).plus(Variable.abs(cylinder.minus(cone))).div(2.0);
                problem.subjectTo(ge(maxVal, 0.0));
            }

            // Dynamics constraints - RK4 integration
            Variable h = dt;
            for (int k = 0; k < N - 1; k++) {
                VariableBlock xk = X.col(k);
                VariableBlock xk1 = X.col(k + 1);

        VariableMatrix k1 = dynamics(new VariableMatrix(xk));
        VariableMatrix k2 = dynamics(new VariableMatrix(xk).plus(k1.times(h.div(2.0))));
        VariableMatrix k3 = dynamics(new VariableMatrix(xk).plus(k2.times(h.div(2.0))));
        VariableMatrix k4 = dynamics(new VariableMatrix(xk).plus(k3.times(h)));

        VariableMatrix rhs = new VariableMatrix(xk)
            .plus(
                k1.plus(k2.times(2.0)).plus(k3.times(2.0)).plus(k4).times(h.div(6.0)));
        problem.subjectTo(eq(xk1, rhs));
            }

            // Require final position is at target
            SimpleMatrix targetPos = new SimpleMatrix(3, 1, true, new double[] { TARGET_X, TARGET_Y, TARGET_Z });
            problem.subjectTo(eq(p.col(N - 1), targetPos));

            // Require final velocity is downward
            problem.subjectTo(lt(vz.get(N - 1), 0.0));

            // Minimize initial velocity magnitude
            problem.minimize(v0WrtShooter.T().times(v0WrtShooter));

            // Solve
            problem.solve(new Options().withDiagnostics(false));

            // Extract solution
            SimpleMatrix v0 = v0WrtShooter.value();
            double v0x = v0.get(0, 0);
            double v0y = v0.get(1, 0);
            double v0z = v0.get(2, 0);

            double velocity = Math.sqrt(v0x * v0x + v0y * v0y + v0z * v0z);
            double pitch = Math.atan2(v0z, Math.hypot(v0x, v0y));
            double yaw = Math.atan2(v0y, v0x);

            // Convert launch velocity to flywheel angular velocity: omega = v / r
            double flywheelOmega = velocity / FLYWHEEL_RADIUS; // rad/s

            return new ShootingSolution(
                    Units.RadiansPerSecond.of(flywheelOmega), Units.Radians.of(pitch), new Rotation2d(yaw));
        }
    }

    /**
     * Computes the state derivative for the ball dynamics.
     *
     * <p>
     * State x = [x, y, z, vx, vy, vz]
     *
     * <pre>
     * x' = vx
     * y' = vy
     * z' = vz
     * [vx']   [ 0]
     * [vy'] = [ 0] - F_D/m * v_hat - F_L/m * (v x omega)
     * [vz']   [-g]
     * </pre>
     */
    private static VariableMatrix dynamics(VariableMatrix x) {
        VariableBlock vel = x.get(new Slice(3, 6), Slice.__);

        // v^2 = v * v (scalar)
        Variable v2 = vel.T().times(vel).get(0, 0);
        // |v|
        Variable vNorm = Variable.sqrt(v2);
        // unit velocity vector
        VariableMatrix vHat = vel.div(vNorm);

        // Drag force: F_D = 1/2p|v|^2C_D A
        Variable fDrag = v2.times(0.5 * RHO * C_D * BALL_AREA);

        // Magnus/Lift force: F_L = 1/2p|v|C_L A
        Variable fLift = vNorm.times(0.5 * RHO * C_L * BALL_AREA);

        // v x omega (cross product of velocity with constant angular velocity)
        VariableMatrix crossResult = crossVarConst(vel, OMEGA);

        // Gravity
        SimpleMatrix negG = new SimpleMatrix(3, 1, true, new double[] { 0.0, 0.0, -G });

        // Acceleration: -g - F_D/m * v_hat - F_L/m * (v x omega)
        VariableMatrix dragAccel = vHat.times(fDrag.div(BALL_MASS));
        VariableMatrix magnusAccel = crossResult.times(fLift.div(BALL_MASS));
        VariableMatrix accel = dragAccel.unaryMinus().minus(magnusAccel).plus(negG);

        // Return [velocity; acceleration]
        return VariableMatrix.block(
                new VariableMatrix[][] { { new VariableMatrix(vel) }, { accel } });
    }

    /** Cross product of a Variable vector (3x1) with a constant double vector. */
    private static VariableMatrix crossVarConst(VariableBlock a, double[] b) {
        Variable c0 = a.get(1, 0).times(b[2]).minus(a.get(2, 0).times(b[1]));
        Variable c1 = a.get(2, 0).times(b[0]).minus(a.get(0, 0).times(b[2]));
        Variable c2 = a.get(0, 0).times(b[1]).minus(a.get(1, 0).times(b[0]));
        return new VariableMatrix(new Variable[][] { { c0 }, { c1 }, { c2 } });
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
