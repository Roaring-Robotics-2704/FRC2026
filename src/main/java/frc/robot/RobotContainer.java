// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveTuningCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.objectDetection.ObjectDetectionConstants;
import frc.robot.subsystems.objectDetection.ObjectDetectionIO;
import frc.robot.subsystems.objectDetection.ObjectDetectionIOReal;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructureStates.WantedState;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.hopper.HopperIO;
import frc.robot.subsystems.superstructure.hopper.HopperIOReal;
import frc.robot.subsystems.superstructure.hopper.HopperIOSim;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.subsystems.superstructure.intake.IntakeIOReal;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOGreyT;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
//import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private SwerveDriveSimulation driveSimulation = null;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;

    private final Vision vision;
    private final ObjectDetection objectDetection;

    // SuperStructure
    private final SuperStructure superStructure;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        RobotState.getInstance(); // Ensure RobotState is initialized
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(DriveConstants.frontLeftConfig),
                        new ModuleIOTalonFXReal(DriveConstants.frontRightConfig),
                        new ModuleIOTalonFXReal(DriveConstants.backLeftConfig),
                        new ModuleIOTalonFXReal(DriveConstants.backRightConfig));
                intake = new Intake(new IntakeIOReal());
                hopper = new Hopper(new HopperIOReal());
                vision = new Vision(
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));
                objectDetection = new ObjectDetection(new ObjectDetectionIOReal(ObjectDetectionConstants.cameraName,
                        ObjectDetectionConstants.cameraToRobotTransform));
                shooter = new Shooter(new ShooterIOGreyT(), () -> Feet.of(0)); // TODO: Add distance supplier
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));

                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(DriveConstants.frontLeftConfig, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(DriveConstants.frontRightConfig, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(DriveConstants.backLeftConfig, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(DriveConstants.backRightConfig, driveSimulation.getModules()[3]));
                intake = new Intake(new IntakeIO() {
                });
                hopper = new Hopper(new HopperIOSim());

                vision = new Vision(
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(camera1Name, robotToCamera1,
                                driveSimulation::getSimulatedDriveTrainPose));
                objectDetection = new ObjectDetection(new ObjectDetectionIO() {
                });
                shooter = new Shooter(new ShooterIO() {
                }, () -> Feet.of(0)); // TODO: Add distance supplier
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                intake = new Intake(new IntakeIO() {
                });
                hopper = new Hopper(new HopperIO() {
                });

                vision = new Vision(new VisionIO() {
                }, new VisionIO() {
                });
                objectDetection = new ObjectDetection(new ObjectDetectionIO() {
                });

                shooter = new Shooter(new ShooterIO() {
                }, () -> Feet.of(0)); // TODO: Add distance supplier
                break;
        }

        // Set up superstructure
        superStructure = new SuperStructure(intake, hopper, shooter);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));
        vision.setDefaultCommand(vision.idle());
        objectDetection.setDefaultCommand(objectDetection.idle());

        // Lock to 0 deg when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> Rotation2d.kZero));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0 deg when B button is pressed

        // Lock to 0 deg when A button is held
        controller
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> Rotation2d.kZero));

        controller.leftTrigger().onTrue(superStructure.goToState(WantedState.INTAKE));
        controller.rightTrigger().onTrue(superStructure.goToState(WantedState.SHOOT));

        // Reset gyro to 0 deg when B button is pressed

        controller
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(RobotState.getInstance().getPose().getTranslation(),
                                                Rotation2d.kZero)),
                                drive)
                                .ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Return the selected autonomous command
        return autoChooser.get();
    }

    /**
     * Use this to pass the calibration command to the main {@link Robot} class.
     *
     * @return the command to run in calibration
     */
    public Command getCalibrationCommand() {
        return Commands.sequence(superStructure.goToState(WantedState.INTAKE_CALIBRATE_IN),
                superStructure.goToState(WantedState.INTAKE_CALIBRATE_OUT));
    }

    /** Reset the simulation field. */
    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    /** Update the simulation. */
    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
