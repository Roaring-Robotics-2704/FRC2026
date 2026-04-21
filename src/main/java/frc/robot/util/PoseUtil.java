// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public final class PoseUtil {
    private PoseUtil() {
        // Prevent instantiation
    }

    public static double distance(Pose2d pose1, Pose2d pose2) {
        double y =  Math.abs((pose2.getY() - pose1.getY()));
        double x = Math.abs((pose2.getX() - pose1.getX()));
        return Math.abs(Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2)));
    }
}
