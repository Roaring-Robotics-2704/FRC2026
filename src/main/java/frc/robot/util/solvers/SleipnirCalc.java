// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.*;
import static org.wpilib.math.optimization.Constraints.*;

import org.wpilib.math.optimization.Problem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;


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
    private static final double kGravity = 9.81; // m/s^2
    private static final double kAirDensity = 1.225; // kg/m^3
    private static final double kDragCoefficient = 0.47; // dimensionless
    private static final double kMagnusCoefficient = 0.1; // dimensionless
    private static final double kBallRadius = 0.0762; // m (3 inches)
    private static final double kBallMass = 0.227; // kg (0.5 lbs)

    @Override
    public ShootingSolution getShootingSolution(Pose2d robotPose, Twist2d robotVelocity) {
        Problem problem = new Problem();
        return null;
    }
        

}
