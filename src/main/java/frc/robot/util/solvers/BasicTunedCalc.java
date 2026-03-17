// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.LinearFilter;

// x = v0 * cos(theta) * ( (v0 * sin(theta) + sqrt((v0 * sin(theta))^2 - 2 * g * (hf - hi))) / g )

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.PoseUtil;

/** Add your docs here. */
public class BasicTunedCalc {
    private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
    private final LinearFilter driveAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
    private Pose2d blueHubPose = new Pose2d(4.625, 4.03, Rotation2d.fromDegrees(0));

    double lastHoodAngle = 0;
    double lastDriveAngle = 0;

    private double MIN_ANGLE = ShooterConstants.MIN_ANGLE.in(Degrees);
    /** Basic interpolated map for shooter calculations. */
    public BasicTunedCalc() {
        // Example data points (distance in feet, angle in degrees)d
        hoodAngleMap.put(3.0, MIN_ANGLE);
        flywheelSpeedMap.put(1.0, 2800.0);
        timeOfFlightMap.put(1.0, 1.1);

        hoodAngleMap.put(6.0, MIN_ANGLE+5);
        flywheelSpeedMap.put(2.0, 3500.0);
        timeOfFlightMap.put(2.0, 0.62);

        hoodAngleMap.put(9.0, MIN_ANGLE+10);
        flywheelSpeedMap.put(3.0, 4000.0);
        timeOfFlightMap.put(3.0, 4.0);

        hoodAngleMap.put(12.0, MIN_ANGLE+15);
        flywheelSpeedMap.put(4.0, 4500.0);
        timeOfFlightMap.put(4.0, 5.0);

        hoodAngleMap.put(15.0, MIN_ANGLE+20);
        flywheelSpeedMap.put(5.0, 5000.0);
        timeOfFlightMap.put(5.0, 6.0);

    }

    public ShootingSolution getShootingSolution() {
        Pose2d robotPose = RobotState.getInstance().getOdometryPose();
        Pose2d shooterPose = robotPose.transformBy(new Transform2d(VisionConstants.robotToCamera0.getMeasureX(),VisionConstants.robotToCamera0.getMeasureY(), VisionConstants.robotToCamera0.getRotation().toRotation2d()));
        
        
        Distance distancetoBlue = Meters.of(PoseUtil.distance(shooterPose, blueHubPose));
        Distance distancetoRed = Meters.of(PoseUtil.distance(shooterPose, FlippingUtil.flipFieldPose(blueHubPose)));
        Pose2d hubPose = blueHubPose;
        if (distancetoRed.in(Meters) < distancetoBlue.in(Meters)) {
            hubPose = FlippingUtil.flipFieldPose(blueHubPose);
        }
        Distance distance = Meters.of(PoseUtil.distance(shooterPose, hubPose));
        Angle hoodAngle = Degrees.of(hoodAngleMap.get(distance.in(Feet)));
        AngularVelocity flywheelVelocity = RPM.of(flywheelSpeedMap.get(distance.in(Feet)));
        hoodAngle = Degrees.of(hoodAngleFilter.calculate(hoodAngle.in(Degrees)));

        Pose2d robotPoseLookAhead = RobotState.getInstance().getLookaheadPose(timeOfFlightMap.get(distance.in(Meters)));
        double dx = blueHubPose.getX() - shooterPose.getX();
        double dy = blueHubPose.getY() - shooterPose.getY();
        double targetAngle = Math.atan2(dy, dx);
        targetAngle = driveAngleFilter.calculate(targetAngle);

        return new ShootingSolution(flywheelVelocity, hoodAngle, Rotation2d.fromRadians(targetAngle), distance);
    }

    /** A record to hold the shooting solution. */
    public record ShootingSolution(AngularVelocity flywheelVelocity, Angle hoodAngle, Rotation2d robotAngle, Distance distance) {
    }
}
