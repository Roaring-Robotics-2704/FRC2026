// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public final class PoseUtil {
    private PoseUtil() {
        // Prevent instantiation
    }

    public static double distance(Pose2d pose1, Pose2d pose2) {
        return Math.abs(pose1.getTranslation().getDistance(pose2.getTranslation()));
    }
}
