// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ObjectDetectionIO {

    /** Object Detection Inputs. */
    @AutoLog
    public static class ObjectDetectionIOInputs {
        public TargetObservation[] targetObservations = new TargetObservation[0];
        
        boolean isConnected = false;
        
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    public void updateInputs(ObjectDetectionIOInputs inputs) {
    }

}
