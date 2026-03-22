// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.LED.LEDManager;
import frc.robot.subsystems.LED.LEDManager.LEDState;
import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.hopper.Hopper.HopperState;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.Kicker;
import frc.robot.util.tunables.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;

public class AutoCommands {
    static Timer clock = new Timer();
    private static final LoggedTunableNumber autoDriveLaunchKp = new LoggedTunableNumber("AutoCommands/Launching/kP",
            8.0);
    private static final LoggedTunableNumber autoDriveLaunchKd = new LoggedTunableNumber("AutoCommands/Launching/kD",
            0.5);

    public static Command followTrajectory(String name, Drive drive, boolean start) {
        Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
        if (trajectoryOptional.isPresent()) {
            Trajectory<SwerveSample> trajectory = trajectoryOptional.get();
            return Commands.sequence(
                    start
                            ? AutoCommands.resetPose(
                                    trajectory.getInitialSample(AllianceFlipUtil.shouldFlip()).get().getPose())
                            : Commands.none(),
                    new DriveTrajectory(trajectory, drive));
        } else {
            throw new RuntimeException("Choreo Trajectory Not Found: " + name);
        }
    }

    public static Command followTrajectoryWhileAiming(
            String name, Drive drive, boolean start, BooleanSupplier shouldAim) {
        Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
        if (trajectoryOptional.isPresent()) {
            Trajectory<SwerveSample> trajectory = trajectoryOptional.get();
            return Commands.sequence(
                    start
                            ? AutoCommands.resetPose(
                                    trajectory.getInitialSample(AllianceFlipUtil.shouldFlip()).get().getPose())
                            : Commands.none(),
                    new DriveTrajectory(
                            trajectory,
                            drive));
        } else {
            throw new RuntimeException("Choreo Trajectory Not Found: " + name);
        }
    }

    public static Command index(Hopper hopper, Kicker kicker, Shooter shooter, Intake intake, LEDManager leds,
            Time shootTime) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    clock.reset();
                    clock.start();
                }),
                Commands.startEnd(
                        () -> {
                            hopper.setDesiredState(HopperState.FEEDING);
                            kicker.setKickerVoltage(12);
                            shooter.setDesiredState(ShooterState.SHOOTING);
                            leds.setPattern(LEDState.SHOOTING);
                        },
                        () -> {
                            hopper.setDesiredState(HopperState.IDLE);
                            kicker.setKickerVoltage(-1);
                            shooter.setDesiredState(ShooterState.IDLE);
                            leds.setPattern(LEDState.IDLE);
                        },
                        hopper,
                        kicker).until(() -> clock.hasElapsed(shootTime)));
    }

    public static boolean xCrossed(double xPosition, boolean towardsCenter) {
        Pose2d robotPose = RobotState.getInstance().getPose();
        if (AllianceFlipUtil.shouldFlip()) {
            if (towardsCenter) {
                return robotPose.getX() < FieldConstants.fieldLength - xPosition;
            } else {
                return robotPose.getX() > FieldConstants.fieldLength - xPosition;
            }
        } else {
            if (towardsCenter) {
                return robotPose.getX() > xPosition;
            } else {
                return robotPose.getX() < xPosition;
            }
        }
    }

    public static boolean yCrossed(double yPosition, boolean towardsLeft) {
        Pose2d robotPose = RobotState.getInstance().getPose();
        if (AllianceFlipUtil.shouldFlip()) {
            if (towardsLeft) {
                return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
            } else {
                return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
            }
        } else {
            if (towardsLeft) {
                return robotPose.getY() > yPosition;
            } else {
                return robotPose.getY() < yPosition;
            }
        }
    }

    public static boolean withinTolerance(
            Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
        Pose2d robotPose = RobotState.getInstance().getPose();
        return robotPose.getTranslation()
                .getDistance(AllianceFlipUtil.apply(target.getTranslation())) < translationalTolerance
                && Math.abs(
                        robotPose
                                .getRotation()
                                .minus(AllianceFlipUtil.apply(target.getRotation()))
                                .getRadians()) < rotationalTolerance.getRadians();
    }

    public static Command waitUntilXCrossed(double xPosition, boolean towardsCenter) {
        return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenter));
    }

    public static Command waitUntilYCrossed(double yPosition, boolean towardsLeft) {
        return Commands.waitUntil(() -> yCrossed(yPosition, towardsLeft));
    }

    public static Command waitUntilWithinTolerance(
            Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
        return Commands.waitUntil(
                () -> withinTolerance(target, translationalTolerance, rotationalTolerance));
    }

    public static Command waitUntilWithinTolerance(
            Supplier<Pose2d> target, double translationalTolerance, Rotation2d rotationalTolerance) {
        return Commands.waitUntil(
                () -> withinTolerance(target.get(), translationalTolerance, rotationalTolerance));
    }

    public static Command driveToPose(Drive drive, Supplier<Pose2d> target) {
        return new DriveToPose(drive, () -> AllianceFlipUtil.apply(target.get()));
    }

    public static Command driveToPoseWhileAiming(Drive drive, Supplier<Pose2d> target) {
        return new DriveToPose(
                drive,
                () -> AllianceFlipUtil.apply(target.get()));
    }

    /**
     * Resets pose accounting for alliance color.
     *
     * @param pose
     *            Pose to reset to.
     */
    public static Command resetPose(Pose2d pose) {
        return Commands.runOnce(
                () -> {
                    RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose));
                });
    }

    public static Command resetPose(Supplier<Pose2d> pose) {
        return Commands.runOnce(
                () -> {
                    RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose.get()));
                });
    }

}
