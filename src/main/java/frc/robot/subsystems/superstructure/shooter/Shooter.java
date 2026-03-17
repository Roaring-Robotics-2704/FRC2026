// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MAX_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_IDLE;

import frc.robot.util.solvers.BasicTunedCalc;
import frc.robot.util.solvers.BasicTunedCalc.ShootingSolution;
import frc.robot.util.tunables.LoggedTunableNumber;

/** The shooter subsystem. */
public class Shooter extends SubsystemBase {

    private ShooterState currentState = ShooterState.STATIONARY;
    private ShooterState desiredState = ShooterState.IDLE;

    private Angle hoodAngle;
    private AngularVelocity flywheelVelocity;
    private final BasicTunedCalc solver = new BasicTunedCalc();

    private final ShooterIO io;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunableNumber shootP = new LoggedTunableNumber("Shooter/ShootP");
    private static final LoggedTunableNumber shootD = new LoggedTunableNumber("Shooter/ShootD");

    private static final LoggedTunableNumber shootS = new LoggedTunableNumber("Shooter/ShootS");
    private static final LoggedTunableNumber shootV = new LoggedTunableNumber("Shooter/ShootV");
    private static final LoggedTunableNumber shootA = new LoggedTunableNumber("Shooter/ShootA");
    private AngularVelocity target = ShooterConstants.SHOOTER_TARGET;
    private boolean hoodOverride = false;
    private boolean flywheelOverride = false;


    static {
        shootP.initDefault(ShooterConstants.SHOOTER_KP);
        shootD.initDefault(ShooterConstants.SHOOTER_KD);

        shootS.initDefault(ShooterConstants.SHOOTER_KS);
        shootV.initDefault(ShooterConstants.SHOOTER_KV);
        shootA.initDefault(ShooterConstants.SHOOTER_KA);
    }

    private Rotation2d robotAngle = Rotation2d.fromDegrees(0    );

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

        if(shootP.hasChanged(hashCode()) || shootD.hasChanged(hashCode()) ||
            shootS.hasChanged(hashCode()) || shootV.hasChanged(hashCode()) || shootA.hasChanged(hashCode())) {
            io.setPID(shootP.get(), 0, shootD.get(), shootS.get(), shootV.get(), shootA.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        ShootingSolution solution = solver.getShootingSolution();
        hoodAngle = solution.hoodAngle();
        flywheelVelocity = solution.flywheelVelocity();
        robotAngle = solution.robotAngle();

        Logger.recordOutput("Shooter/CalculatedHoodAngle", hoodAngle);
        Logger.recordOutput("Shooter/CalculatedFlywheelVelocity", flywheelVelocity);
        Logger.recordOutput("Shooter/CalculatedRobotAngle", robotAngle);
        Logger.recordOutput("Shooter/Distance", solution.distance());
        Logger.recordOutput("Shooter/DesiredState", desiredState.toString());
        Logger.recordOutput("Shooter/CurrentState", currentState.toString());
        hoodAngle = Degrees.of(MathUtil.clamp(hoodAngle.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));
        if (currentState != desiredState) {
            switch (desiredState) {
                case STATIONARY:
                    io.setFlywheelVelocity(RPM.of(0));
                    io.setHoodAngle(MIN_ANGLE);
                    ;
                    break;
                case IDLE:
                    io.setFlywheelVelocity(SHOOTER_IDLE);
                    io.setHoodAngle(MIN_ANGLE);
                    break;
                case SHOOTING:
                    io.setHoodAngle(hoodAngle);
                    io.setFlywheelVelocity(target);
                    break;
                default:
                    break;
            }
            if (inputs.atTargetAngle && inputs.atTargetVelocity) {
                currentState = desiredState;
            }

        }

        // This method will be called once per scheduler run
    }

    /**
     * Sets the desired state of the shooter.
     *
     * @param state The desired ShooterState.
     */
    public void setDesiredState(ShooterState state) {

        this.desiredState = state;

    }

    /** Returns the current state of the shooter.
     *
     * @return The current ShooterState.
     */
    public ShooterState getCurrentState() {
        return currentState;
    }

    /** Checks if the shooter is at the desired state.
     *
     * @return true if the current state matches the desired state, false otherwise.
     */

    public boolean isAtDesiredState() {
        return currentState == desiredState;
    }

    /** Possible states for the shooter subsystem. */
    public enum ShooterState {
        STATIONARY, IDLE, SHOOTING
    }

    public Rotation2d getWantedRobotAngle() {
        return robotAngle;
    }

    public void incrementFlywheelSpeed(double rpm) {
        flywheelOverride = true;
        target = target.plus(RPM.of(rpm));
    }

    public void incrementHoodAngle(double increment) {
        hoodOverride = true;
        hoodAngle = hoodAngle.plus(Degrees.of(increment));
    }

    public void resetOverrides() {
        hoodOverride = false;
        flywheelOverride = false;
    }

}
