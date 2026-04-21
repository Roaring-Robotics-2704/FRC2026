// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.filter.LinearFilter;

// x = v0 * cos(theta) * ( (v0 * sin(theta) + sqrt((v0 * sin(theta))^2 - 2 * g * (hf - hi))) / g )

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
    private final InterpolatingDoubleTreeMap hoodPercentMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private final LinearFilter hoodPercentFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopTimeSeconds));
    private final Pose2d blueHubPose = new Pose2d(4.625, 4.03, Rotation2d.fromDegrees(0));

    double lastHoodAngle = 0;
    double lastDriveAngle = 0;

    /** Basic interpolated map for shooter calculations. */
    public BasicTunedCalc() {
        // Example data points (distance in meters, angle in degrees
        hoodPercentMap.put(1.38, 0.0);// 6
        flywheelSpeedMap.put(1.38, 2550.0);
        timeOfFlightMap.put(1.38, 0.9);

        hoodPercentMap.put(2.0, 10.0);// 9
        flywheelSpeedMap.put(2.0, 2850.0);
        timeOfFlightMap.put(2.0, 1.26);

        hoodPercentMap.put(3.0, 15.0);
        flywheelSpeedMap.put(3.0, 3200.0);
        timeOfFlightMap.put(3.0, 1.25);

        hoodPercentMap.put(4.0, 20.0);
        flywheelSpeedMap.put(4.0, 3430.0);
        timeOfFlightMap.put(4.0, 1.25);

        hoodPercentMap.put(4.8, 22.0);
        flywheelSpeedMap.put(4.8, 3600.0);
        timeOfFlightMap.put(4.8, 1.25);

        // hoodPercentMap.put(15.0, 9.0);
        // flywheelSpeedMap.put(5.0, 5000.0);
        // timeOfFlightMap.put(5.0, 6.0);

    }

    public ShootingSolution getShootingSolution() {
        Pose2d robotPose = RobotState.getInstance().getPose();
        Logger.recordOutput("Calc/RobotPose", robotPose);
        Pose2d shooterPose = robotPose.transformBy(new Transform2d(VisionConstants.robotToCamera0.getMeasureX(),
                 VisionConstants.robotToCamera0.getMeasureY(),
                 VisionConstants.robotToCamera0.getRotation().toRotation2d()));
        Logger.recordOutput("Calc/ShooterPose", shooterPose);
        Distance distancetoBlue = Meters.of(PoseUtil.distance(robotPose, blueHubPose));
        Distance distancetoRed = Meters.of(PoseUtil.distance(robotPose, FlippingUtil.flipFieldPose(blueHubPose)));
        Pose2d hubPose = blueHubPose;
        if (distancetoRed.in(Meters) < distancetoBlue.in(Meters)) {
            hubPose = FlippingUtil.flipFieldPose(blueHubPose);
        }
        Logger.recordOutput("Calc/HubPose", hubPose);
        double distance = PoseUtil.distance(robotPose, hubPose);
        Logger.recordOutput("Calc/Distance", distance);
        double hoodPercent = hoodPercentMap.get(distance);
        AngularVelocity flywheelVelocity = RPM.of(flywheelSpeedMap.get(distance));
        hoodPercent = hoodPercentFilter.calculate(hoodPercent);

//        Pose2d robotPoseLookAhead = RobotState.getInstance().getLookaheadPose(timeOfFlightMap.get(distance));
//        double dx = blueHubPose.getX() - robotPose.getX();
//        double dy = blueHubPose.getY() - robotPose.getY();
//        double targetAngle = Math.atan2(dy, dx);
        Translation2d delta = hubPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d robotAngle = delta.getAngle();
//        targetAngle = driveAngleFilter.calculate(targetAngle);

        return new ShootingSolution(flywheelVelocity, hoodPercent, robotAngle, distance);
    }

    /** A record to hold the shooting solution. */
    public class ShootingSolution {
        public final AngularVelocity flywheelVelocity;
        public final double hoodPercent;
        public final Rotation2d driveAngle;
        public final double distance;

        public ShootingSolution(AngularVelocity flywheelVelocity, double hoodPercent, Rotation2d driveAngle,
                Double distance) {
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPercent = hoodPercent;
            this.driveAngle = driveAngle;
            this.distance = distance;
        }

        public double flywheelVelocity() {
            return flywheelVelocity.in(RPM);
        }

        public double hoodPercent() {
            return hoodPercent;
        }

        public Rotation2d driveAngle() {
            return driveAngle;
        }

        public double distance() {
            return distance;
        }
    }
}
