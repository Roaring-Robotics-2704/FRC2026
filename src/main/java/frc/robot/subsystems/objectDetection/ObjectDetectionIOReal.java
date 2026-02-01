// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;


import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class ObjectDetectionIOReal implements ObjectDetectionIO {
    PhotonCamera camera;
    Translation3d cameraToRobotTransform;

    public ObjectDetectionIOReal(String cameraName, Translation3d cameraToRobotTransform) {
        camera = new PhotonCamera(cameraName);
        this.cameraToRobotTransform = cameraToRobotTransform;
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        inputs.isConnected = camera.isConnected();
        ArrayList<TargetObservation> targets = new ArrayList<>();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            for (PhotonPipelineResult result : results) {
                if (result.hasTargets()) {

                    for (PhotonTrackedTarget target : result.getTargets()) {
                        
                        targets.add(new TargetObservation(
                                Rotation2d.fromDegrees(target.getPitch()),
                                Rotation2d.fromDegrees(target.getYaw())));

                    }

                }
            }

        }

    }
}
