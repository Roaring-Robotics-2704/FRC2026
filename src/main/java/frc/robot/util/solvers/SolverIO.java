// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public interface SolverIO {

    default ShootingSolution getShootingSolution() {
        return new ShootingSolution(RPM.of(0), Degrees.of(0), Rotation2d.kZero);
    }

    /** A record to hold the shooting solution. */
    public record ShootingSolution(AngularVelocity flywheelVelocity, Angle hoodAngle, Rotation2d robotAngle) {
    }
}
