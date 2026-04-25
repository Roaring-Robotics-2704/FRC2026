// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.auto.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
//import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.ChoreoTraj;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.LED.LEDManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.Kicker;
import frc.robot.subsystems.superstructure.climber.Climber;
// import frc.robot.subsystems.superstructure.hook.Hook;
// import frc.robot.subsystems.superstructure.hook.Hook.HookState;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//@RequiredArgsConstructor
@SuppressWarnings("unused")
public class AutoBuilder {
    private final Drive drive;
    private final Intake intake;
    private final Hopper hopper;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Climber climber;
    private LEDManager manager;

    private final AutoFactory autoFactory;

    // private final Supplier<List<AutoQuestionResponse>> responses;

    public static final double outpostIntakeTime = 3.0;
    public static final double neutralZoneIntakeTimeFirst = 3.0;
    public static final double neutralZoneIntakeTimeOther = 4.0;
    public static final double launchTime = 4.0;

    public AutoBuilder(Drive drive, Intake intake, Hopper hopper, Kicker kicker, Shooter shooter, Climber climber, LEDManager manager) {
        this.drive = drive;
        this.intake = intake;
        this.hopper = hopper;
        this.kicker = kicker;
        this.shooter = shooter;
        this.climber = climber;
        this.manager = manager;

        autoFactory = new AutoFactory(
                () -> RobotState.getInstance().getPose(), // A function that returns the current robot pose
                (pose) -> RobotState.getInstance().resetPose(pose), 
                // A function that resets the current robot pose to the provided Pose2d
                drive::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                drive // The drive subsystem
        );
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> sendableChooser = new SendableChooser<Command>();

        sendableChooser.addOption("Shoot only", minMovementShoot().cmd().withName("Shoot only"));
        sendableChooser.addOption("Shoot, climb", shootAndClimb().cmd().withName("Shoot, climb"));
        sendableChooser.addOption("Shoot, collect, climb", shootCollectClimb().cmd().withName("Shoot, collect, climb"));
        sendableChooser.addOption("Shoot, collect, climb (mirrored)", shootCollectClimbMirrored().cmd().withName("Shoot, collect, climb (mirrored)"));
        sendableChooser.addOption("Shoot, collect, pass, collect, pass", shootCollectPass().cmd().withName("Shoot, collect, pass, collect, pass"));
        sendableChooser.addOption("Shoot, collect, shoot", shootCollectShoot().cmd().withName("Shoot, collect, shoot"));
        sendableChooser.addOption("Shoot, collect, shoot (mirrored)", shootCollectShootMirrored().cmd().withName("Shoot, collect, shoot (mirrored)"));
        sendableChooser.addOption("Shoot, collect, shoot, climb", shootCollectShootClimb().cmd().withName("Shoot, collect, shoot, climb"));
        sendableChooser.addOption("Shoot, collect, shoot, climb (mirrored)", shootCollectShootClimbMirrored().cmd().withName("Shoot, collect, shoot, climb (mirrored)"));
        sendableChooser.addOption("Shoot, depot, climb", shootDepotClimb().cmd().withName("Shoot, depot, climb"));
        sendableChooser.addOption("Shoot, depot, shoot", shootDepotShoot().cmd().withName("Shoot, depot, shoot"));
        sendableChooser.addOption("The cooler shoot, depot, shoot", theCoolerShootDepotShoot().cmd().withName("The cooler shoot, depot, shoot"));
        sendableChooser.addOption("AAA the cooler shoot, depot, shoot", aaaTheCoolerShootDepotShoot().cmd().withName("AAA the cooler shoot, depot, shoot"));

        return sendableChooser;
    }

    /*@Override
    public void periodic() {
        Logger.recordOutput("Auto/", value);
    }*/

    public Command shootPreloadCommandSequence() {
        return Commands.sequence(
                AutoCommands.index(hopper, kicker, shooter, intake, manager, Seconds.of(7)).withName("Auto Shoot Preload")
                ).withName("Shoot Preloaded Fuel");
    }

    public Command shootCollectedFuelCommandSequence() {
        return Commands.sequence(
                AutoCommands.index(hopper, kicker, shooter, intake, manager, Seconds.of(12)).withName("Auto Shoot Collected Fuel")
                ).withName("Shoot Collected Fuel");
    }

    public Command climbCommandSequence() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    climber.enablehook(true);
                }),
                climber.setClimber(6).withTimeout(4),
                Commands.runOnce(() -> {
                    climber.setClimber(-6);
                }),
            Commands.runOnce(() -> {
                climber.enablehook(true);
            }));
    }

    public Command retractIntakeCommandSequence() {
        return intake.retract().withTimeout(3).withName("Retract Intake Auto");
    }

    public Command extendIntakeCommandSequence() {
        return intake.intake().withName("Extend Intake Auto");
    }

    public Command runAutoTrajectory(AutoTrajectory trajectory) {
        return Commands.sequence(
                trajectory.cmd(),
                Commands.runOnce(drive::stopWithX)).withName("Run Auto Trajectory: " + trajectory.toString());
    }

    public AutoRoutine minMovementShoot() {
        AutoRoutine routine = autoFactory.newRoutine("MinMovementShoot");

        // Load the routine's trajectories
        AutoTrajectory minMovementShootTrajectory = ChoreoTraj.MinMovementShoot.asAutoTraj(routine);
        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(Commands.sequence(
            minMovementShootTrajectory.resetOdometry(),
            minMovementShootTrajectory.cmd()));
        // Starting at the event marker named "intake", run the intake
        minMovementShootTrajectory.atTime("Shoot Preloaded Fuel").onTrue(shootPreloadCommandSequence());
        return routine;
    }

    public AutoRoutine shootAndClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootAndClimb");

        AutoTrajectory shootAndClimb$0 = ChoreoTraj.ShootAndClimb$0.asAutoTraj(routine);
        AutoTrajectory shootAndClimb$1 = ChoreoTraj.ShootAndClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootAndClimb$0.resetOdometry(),
                        runAutoTrajectory(shootAndClimb$0),
                        runAutoTrajectory(shootAndClimb$1)));

        shootAndClimb$0.atTime("Shoot Preloaded Fuel").onTrue(shootPreloadCommandSequence());

        shootAndClimb$1.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectClimb");

        AutoTrajectory shootCollectClimb$0 = ChoreoTraj.ShootCollectClimb$0.asAutoTraj(routine);
        AutoTrajectory shootCollectClimb$1 = ChoreoTraj.ShootCollectClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectClimb$0.resetOdometry(),
                        runAutoTrajectory(shootCollectClimb$0),
                        runAutoTrajectory(shootCollectClimb$1)));

        shootCollectClimb$0.atTime("Shoot Preloaded Fuel").onTrue(shootPreloadCommandSequence());
        shootCollectClimb$1.atTime("Extend Inake").onTrue(extendIntakeCommandSequence());
        shootCollectClimb$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectClimb$1.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectPass() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectPass");

        AutoTrajectory shootCollectPass$0 = ChoreoTraj.ShootCollectPass$0.asAutoTraj(routine);
        AutoTrajectory shootCollectPass$1 = ChoreoTraj.ShootCollectPass$1.asAutoTraj(routine);
        AutoTrajectory shootCollectPass$2 = ChoreoTraj.ShootCollectPass$2.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectPass$0.resetOdometry(),
                        runAutoTrajectory(shootCollectPass$0),
                        runAutoTrajectory(shootCollectPass$1),
                        runAutoTrajectory(shootCollectPass$2)));

        shootCollectPass$0.atTime("Shoot Preloads").onTrue(shootPreloadCommandSequence());
        shootCollectPass$1.atTime("Extend Intake 1").onTrue(extendIntakeCommandSequence());
        shootCollectPass$1.atTime("Pass 1").onTrue(shootCollectedFuelCommandSequence());
        shootCollectPass$2.atTime("Pass 2").onTrue(
                Commands.sequence(
                        shootCollectedFuelCommandSequence(),
                        retractIntakeCommandSequence()));

        return routine;
    }

    public AutoRoutine shootCollectShoot() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectShoot");

        AutoTrajectory shootCollectShoot$0 = ChoreoTraj.ShootCollectShoot$0.asAutoTraj(routine);
        AutoTrajectory shootCollectShoot$1 = ChoreoTraj.ShootCollectShoot$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectShoot$0.resetOdometry(),
                        runAutoTrajectory(shootCollectShoot$0),
                        AutoCommands.index(hopper,kicker,shooter,intake,manager,Seconds.of(5)),
                        runAutoTrajectory(shootCollectShoot$1),
                        AutoCommands.index(hopper,kicker,shooter,intake,manager,Seconds.of(7))));

        shootCollectShoot$0.atTime("Shoot Preloads").onTrue(Commands.none());
        shootCollectShoot$1.atTime("Open Intake").onTrue(extendIntakeCommandSequence());
        shootCollectShoot$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectShoot$1.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());

        return routine;
    }

    public AutoRoutine shootDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootDepotClimb");

        AutoTrajectory shootDepotClimb$0 = ChoreoTraj.ShootDepotClimb$0.asAutoTraj(routine);
        AutoTrajectory shootDepotClimb$1 = ChoreoTraj.ShootDepotClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootDepotClimb$0.resetOdometry(),
                        runAutoTrajectory(shootDepotClimb$0),
                        runAutoTrajectory(shootDepotClimb$1)));

        shootDepotClimb$0.atTime("Shoot").onTrue(shootPreloadCommandSequence());
        shootDepotClimb$1.atTime("Extend Intake").onTrue(extendIntakeCommandSequence());
        shootDepotClimb$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootDepotClimb$1.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine shootDepotShoot() {
        AutoRoutine routine = autoFactory.newRoutine("ShootDepotShoot");

        AutoTrajectory shootDepotShoot$0 = ChoreoTraj.ShootDepotShoot$0.asAutoTraj(routine);
        AutoTrajectory shootDepotShoot$1 = ChoreoTraj.ShootDepotShoot$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootDepotShoot$0.resetOdometry(),
                        runAutoTrajectory(shootDepotShoot$0),
                        //AutoCommands.index(hopper,kicker,shooter,intake,manager,Seconds.of(5)),
                        runAutoTrajectory(shootDepotShoot$1)));

        shootDepotShoot$0.atTime("Shoot 1").onTrue(shootPreloadCommandSequence());
        shootDepotShoot$1.atTime("Extend Intake").onTrue(extendIntakeCommandSequence());
        shootDepotShoot$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootDepotShoot$1.atTime("Shoot 2").onTrue(shootCollectedFuelCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectClimbMirrored() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectClimbMirrored");

        AutoTrajectory shootCollectClimbMirrored$0 = ChoreoTraj.ShootCollectClimbMirrored$0.asAutoTraj(routine);
        AutoTrajectory shootCollectClimbMirrored$1 = ChoreoTraj.ShootCollectClimbMirrored$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectClimbMirrored$0.resetOdometry(),
                        runAutoTrajectory(shootCollectClimbMirrored$0),
                        runAutoTrajectory(shootCollectClimbMirrored$1)));

        shootCollectClimbMirrored$0.atTime("Shoot Preloaded Fuel").onTrue(shootPreloadCommandSequence());
        shootCollectClimbMirrored$1.atTime("Extend Intake").onTrue(extendIntakeCommandSequence());
        shootCollectClimbMirrored$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectClimbMirrored$1.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectShootMirrored() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectShootMirrored");

        AutoTrajectory shootCollectShootMirrored$0 = ChoreoTraj.ShootCollectShootMirrored$0.asAutoTraj(routine);
        AutoTrajectory shootCollectShootMirrored$1 = ChoreoTraj.ShootCollectShootMirrored$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectShootMirrored$0.resetOdometry(),
                        runAutoTrajectory(shootCollectShootMirrored$0),
                        AutoCommands.index(hopper,kicker,shooter,intake,manager,Seconds.of(5)),
                        runAutoTrajectory(shootCollectShootMirrored$1),
                        AutoCommands.index(hopper,kicker,shooter,intake,manager,Seconds.of(7))
                    ));

        shootCollectShootMirrored$0.atTime("Shoot Preloads").onTrue(Commands.none());
        shootCollectShootMirrored$1.atTime("Open Intake").onTrue(extendIntakeCommandSequence());
        shootCollectShootMirrored$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectShootMirrored$1.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectShootClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectShootClimb");

        AutoTrajectory shootCollectShootClimb$0 = ChoreoTraj.ShootCollectShootClimb$0.asAutoTraj(routine);
        AutoTrajectory shootCollectShootClimb$1 = ChoreoTraj.ShootCollectShootClimb$1.asAutoTraj(routine);
        AutoTrajectory shootCollectShootClimb$2 = ChoreoTraj.ShootCollectShootClimb$2.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectShootClimb$0.resetOdometry(),
                        runAutoTrajectory(shootCollectShootClimb$0),
                        runAutoTrajectory(shootCollectShootClimb$1)));

        shootCollectShootClimb$0.atTime("Shoot Preloads").onTrue(shootPreloadCommandSequence());
        shootCollectShootClimb$1.atTime("Open Intake").onTrue(extendIntakeCommandSequence());
        shootCollectShootClimb$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectShootClimb$1.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());
        shootCollectShootClimb$2.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine shootCollectShootClimbMirrored() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectShootClimbMirrored");

        AutoTrajectory shootCollectShootClimbMirrored$0 = ChoreoTraj.ShootCollectShootClimbMirrored$0
                .asAutoTraj(routine);
        AutoTrajectory shootCollectShootClimbMirrored$1 = ChoreoTraj.ShootCollectShootClimbMirrored$1
                .asAutoTraj(routine);
        AutoTrajectory shootCollectShootClimbMirrored$2 = ChoreoTraj.ShootCollectShootClimbMirrored$2
                .asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        shootCollectShootClimbMirrored$0.resetOdometry(),
                        runAutoTrajectory(shootCollectShootClimbMirrored$0),
                        runAutoTrajectory(shootCollectShootClimbMirrored$1)));

        shootCollectShootClimbMirrored$0.atTime("Shoot Preloads").onTrue(shootPreloadCommandSequence());
        shootCollectShootClimbMirrored$1.atTime("Open Intake").onTrue(extendIntakeCommandSequence());
        shootCollectShootClimbMirrored$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        shootCollectShootClimbMirrored$1.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());
        shootCollectShootClimbMirrored$2.atTime("Climb").onTrue(climbCommandSequence());

        return routine;
    }

    public AutoRoutine theCoolerShootDepotShoot()
    {
        AutoRoutine routine = autoFactory.newRoutine("TheCoolerShootDepotShoot");

        AutoTrajectory theCoolerShootDepotShoot$0 = ChoreoTraj.TheCoolerShootDepotShoot$0.asAutoTraj(routine);
        AutoTrajectory theCoolerShootDepotShoot$1 = ChoreoTraj.TheCoolerShootDepotShoot$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        theCoolerShootDepotShoot$0.resetOdometry(),
                        runAutoTrajectory(theCoolerShootDepotShoot$0),
                        runAutoTrajectory(theCoolerShootDepotShoot$1)));
        
        theCoolerShootDepotShoot$0.atTime("Shoot Preloads").onTrue(shootPreloadCommandSequence());
        theCoolerShootDepotShoot$1.atTime("Extend Intake").onTrue(extendIntakeCommandSequence());
        theCoolerShootDepotShoot$1.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        theCoolerShootDepotShoot$1.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());

        return routine;
    }

    public AutoRoutine aaaTheCoolerShootDepotShoot()
    {
        AutoRoutine routine = autoFactory.newRoutine("AAATheCoolerShootDepotShoot");

        AutoTrajectory aaaTheCoolerShootDepotShoot = ChoreoTraj.AAATheCoolerShootDepotShoot.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        aaaTheCoolerShootDepotShoot.resetOdometry(),
                        runAutoTrajectory(aaaTheCoolerShootDepotShoot)));
        
        aaaTheCoolerShootDepotShoot.atTime("Shoot Preloads").onTrue(shootPreloadCommandSequence());
        aaaTheCoolerShootDepotShoot.atTime("Extend Intake").onTrue(extendIntakeCommandSequence());
        aaaTheCoolerShootDepotShoot.atTime("Retract Intake").onTrue(retractIntakeCommandSequence());
        aaaTheCoolerShootDepotShoot.atTime("Shoot Collected Fuel").onTrue(shootCollectedFuelCommandSequence());
        
        return routine;
    }
}
