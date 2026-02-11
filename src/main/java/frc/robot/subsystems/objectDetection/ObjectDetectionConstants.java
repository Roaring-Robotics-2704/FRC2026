// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ObjectDetectionConstants {

    public static final String cameraName = "Intake Cam";
    public static final Transform3d cameraToRobotTransform = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final Distance fuelHeight = Inches.of(5.91 / 2); // Half the height of a fuel cell

}
