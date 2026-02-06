// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.objectDetection.ObjectDetectionConstants.cameraToRobotTransform;
import static frc.robot.subsystems.objectDetection.ObjectDetectionConstants.fuelHeight;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class ObjectDetection extends SubsystemBase {
    private final ObjectDetectionIO io;
    private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();
    private Angle cameraAngley = cameraToRobotTransform.getRotation().getMeasureX();
    private Angle cameraAnglex = cameraToRobotTransform.getRotation().getMeasureY();
    private Distance cameraHeight = cameraToRobotTransform.getTranslation().getMeasureZ();
    private final Supplier<Pose2d> robotPoseSupplier;

    /** Creates a new ObjectDetection. */
    public ObjectDetection(ObjectDetectionIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Object Detection", inputs);
        if (inputs.targetObservations.length > 0) {
            Pose2d robotPose = robotPoseSupplier.get();
            ArrayList<Pose2d> fuelPoses = new ArrayList<>();
            for (ObjectDetectionIO.TargetObservation observation : inputs.targetObservations) {
                Pose2d fuelPose = getFuelPose(
                    observation.pitch().getMeasure(), 
                    observation.yaw().getMeasure(),
                    robotPose);
                fuelPoses.add(fuelPose);
                RobotState.getInstance().addFuelPose(new FuelPoseEstimate(fuelPose, Seconds.of(Timer.getFPGATimestamp())));
            }
            fuelPoses = sortPosesByDistance(fuelPoses);
            Pose2d[] fuelPosesArray = new Pose2d[fuelPoses.size()];
            fuelPosesArray = fuelPoses.toArray(fuelPosesArray);
            Logger.recordOutput("Object Detection/Fuel Poses",  fuelPosesArray);

        }
    }

    /** 
     * Get the pose of a fuel cell relative to the camera.
     * @param tx The horizontal angle to the target.
     * @param ty The vertical angle to the target.
     * @return The pose of the fuel cell relative to the camera.
     */
    public Pose2d getFuelPose(Angle pitch, Angle yaw, Pose2d robotPose) {
        Angle angleToTargetPitch = cameraAngley.plus(pitch);
        Distance distanceY = Inches.of( // Distance straight outwards from the camera
            Math.tan(angleToTargetPitch.in(Radians)) 
            * (cameraHeight.in(Inches) - fuelHeight.in(Inches)));

        Angle angleToTargetYaw = cameraAnglex.plus(yaw);
        Distance distanceX = Inches.of( // Distance side to side from the camera
            Math.tan(angleToTargetYaw.in(Radians)) 
            * distanceY.in(Inches));
        return robotPose.plus(new Transform2d(distanceY, distanceX, robotPose.getRotation()));
    }

    /** 
     * Sorts a list of poses by distance from the robot.
     * @param poses The list of poses to sort.
     * @return The sorted list of poses.
     */
    public ArrayList<Pose2d> sortPosesByDistance(ArrayList<Pose2d> poses) {
        Pose2d robotPose = robotPoseSupplier.get();
        poses.sort((a, b) -> {
            Distance distanceA = Meters.of(new Translation2d(a.getX(), a.getY())
                .getDistance(robotPose.getTranslation()));
            Distance distanceB = Meters.of(new Translation2d(b.getX(), b.getY())
                .getDistance(robotPose.getTranslation()));
            return Double.compare(distanceA.in(Inches), distanceB.in(Inches));
        });
        return poses;
    }
    public static record  FuelPoseEstimate(Pose2d pose, Time timestamp) {
    }
}
