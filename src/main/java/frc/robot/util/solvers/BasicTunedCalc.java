// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;

// x = v0 * cos(theta) * ( (v0 * sin(theta) + sqrt((v0 * sin(theta))^2 - 2 * g * (hf - hi))) / g )

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Robot;
import frc.robot.util.PoseUtil;

/** Add your docs here. */
public class BasicTunedCalc implements SolverIO {
    private static BasicTunedCalc instance;
    private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    private Pose2d blueHubPose = new Pose2d(4.625, 4.03, Rotation2d.fromDegrees(0));
    
    private int rotationsPerMinute = 2800;

    public BasicTunedCalc() {
        // Example data points (distance in meters, angle in degrees)
        hoodAngleMap.put(1.0, 10.0);
        hoodAngleMap.put(2.0, 20.0);
        hoodAngleMap.put(3.0, 30.0);
        hoodAngleMap.put(4.0, 40.0);
        hoodAngleMap.put(5.0, 45.0);
    }


    @Override
    public ShootingSolution getShootingSolution(Pose2d robotPose, ChassisSpeeds robotVelocity) {
        Distance distance = Meters.of(PoseUtil.distance(robotPose, blueHubPose));
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(distance.in(Meters)));
        double dx = blueHubPose.getX() - robotPose.getX();
        double dy = blueHubPose.getY() - robotPose.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));
        return new ShootingSolution(DegreesPerSecond.of(rotationsPerMinute * 360.0 / 60.0), hoodAngle, targetAngle);
    }
}
