// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

//import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.commands.auto.AutoBuilder;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.superstructure.kicker.Kicker;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.climber.ClimberIO;
import frc.robot.subsystems.superstructure.climber.ClimberIOReal;
import frc.robot.subsystems.superstructure.climber.ClimberIOSim;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.hopper.HopperIO;
import frc.robot.subsystems.superstructure.hopper.HopperIOReal;
import frc.robot.subsystems.superstructure.hopper.HopperIOSim;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.IntakeIO;
import frc.robot.subsystems.superstructure.intake.IntakeIOReal;
import frc.robot.subsystems.superstructure.shooter.BasicTunedCalc;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.ShooterIO;
import frc.robot.subsystems.superstructure.shooter.ShooterIOGreyT;
import frc.robot.subsystems.superstructure.hook.Hook;
import frc.robot.subsystems.superstructure.hook.HookIO;
import frc.robot.subsystems.superstructure.hook.HookIOReal;
import frc.robot.subsystems.superstructure.hook.HookIOSim;
import frc.robot.subsystems.vision.Vision;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import frc.robot.subsystems.vision.VisionIO;
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
    public final Drive drive;
    public final Hopper hopper;
    public final Kicker kicker = new Kicker();
    public final Intake intake;
    public final Shooter shooter;
    public final Climber climber;
    //public final Hook hook;

    public final Vision vision;
    // private final FuelPoseEstimator objectDetection;

    // SuperStructure
    // private final SuperStructure superStructure;

    // Controller

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    //

    public boolean noAutoSelected() {
        var selected = autoChooser.getSendableChooser().getSelected();
        return selected == null || selected == "None";
    }

    // Choreo auto factory library
    // private final AutoFactory autoFactory;
    private final AutoBuilder autoBuilder;

    private final LoggedDashboardChooser<Command> testChooser;

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
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                intake = new Intake(new IntakeIOReal());
                hopper = new Hopper(new HopperIOReal());
                climber = new Climber(new ClimberIOReal() {
                });
                //hook = new Hook(new HookIO() {
                //});
                vision = new Vision(
                        new VisionIOPhotonVision(camera0Name, robotToCamera0));

                        
                // objectDetection = new FuelPoseEstimator(new
                // ObjectDetectionIOReal(ObjectDetectionConstants.cameraName,
                // ObjectDetectionConstants.cameraToRobotTransform));
                shooter = new Shooter(new ShooterIOGreyT());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                intake = new Intake(new IntakeIO() {
                });
                hopper = new Hopper(new HopperIOSim());
                climber = new Climber(new ClimberIO() {
                });
                //hook = new Hook(new HookIOSim() {
                //});

                vision = new Vision(
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
                // objectDetection = new FuelPoseEstimator(new ObjectDetectionIO() {
                // });
                shooter = new Shooter(new ShooterIO() {});
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
                });
                // objectDetection = new FuelPoseEstimator(new ObjectDetectionIO() {
                // });

                climber = new Climber(new ClimberIO() {
                });
                //hook = new Hook(new HookIO() {
                //});

                shooter = new Shooter(new ShooterIO() {
                });
                break;
        }

        // Set up superstructure
        // superStructure = new SuperStructure(intake, hopper, kicker, shooter,
        // climber);

        // Set up auto routines
        autoBuilder = new AutoBuilder(drive, intake, hopper, kicker, shooter, climber);
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", autoBuilder.buildAutoChooser());

        Controls.getInstance().configureControls(this);

        testChooser = new LoggedDashboardChooser<>("Test Command");

        // TODO: Fix
        
        // autoFactory = new AutoFactory(
        // ()->RobotState.getInstance().getPose(), // A function that returns the
        // current robot pose
        // (Pose2d pose)->RobotState.getInstance().resetPose(pose), // A function that
        // resets the current robot pose to the provided Pose2d
        // drive::followTrajectory, // The drive subsystem trajectory follower
        // true, // If alliance flipping should be enabled
        // drive // The drive subsystem
        // );

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

    public Command getTestCommand() {
        return testChooser.get();
    }
}
