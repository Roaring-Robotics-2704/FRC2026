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
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.ChoreoTraj;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.Intake.IntakeState;
import frc.robot.subsystems.superstructure.Kicker.Kicker;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.climber.Climber.ClimberState;
import frc.robot.subsystems.superstructure.hook.Hook;
import frc.robot.subsystems.superstructure.hook.Hook.HookState;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.commands.auto.AutoChooser;
import choreo.auto.AutoFactory;

//@RequiredArgsConstructor
@SuppressWarnings("unused")
public class AutoBuilder {
    private final Drive drive;
    private final Intake intake;
    private final Hopper hopper;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Climber climber;
    private final Hook hook;

    private final AutoFactory autoFactory;

    // private final Supplier<List<AutoQuestionResponse>> responses;

    public static final double outpostIntakeTime = 3.0;
    public static final double neutralZoneIntakeTimeFirst = 3.0;
    public static final double neutralZoneIntakeTimeOther = 4.0;
    public static final double launchTime = 4.0;

    public AutoBuilder(Drive drive, Intake intake, Hopper hopper, Kicker kicker, Shooter shooter, Climber climber, Hook hook)
    {
        this.drive = drive;
        this.intake = intake;
        this.hopper = hopper;
        this.kicker = kicker;
        this.shooter = shooter;
        this.climber = climber;
        this.hook = hook;
        
        autoFactory = new AutoFactory(
            ()->RobotState.getInstance().getPose(), // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drive // The drive subsystem
        );
    }


    public Command shootPreloadCommandSequence() {
        return  Commands.sequence(
                    Commands.runOnce( () -> { shooter.setDesiredState(ShooterState.SHOOTING); } ),
                    Commands.waitSeconds(3),
                    Commands.runOnce( () -> { shooter.setDesiredState(ShooterState.IDLE); } )
                );
    }
    public Command shootCollectedFuelCommandSequence() {
        return  Commands.sequence(
                    Commands.runOnce(() -> {shooter.setDesiredState(ShooterState.SHOOTING);}),
                    Commands.waitSeconds(6),
                    Commands.runOnce(() -> { shooter.setDesiredState(ShooterState.IDLE);})  
        );
    }

    public Command climbCommandSequence() {
        return  Commands.sequence(
                    Commands.runOnce( () -> { hook.setDesiredState(HookState.UNPOWERED); } ),
                    Commands.waitUntil( hook::isAtDesiredState ),

                    Commands.runOnce( () -> { climber.setDesiredState(ClimberState.TOP); } ),
                    Commands.waitUntil(climber::isAtDesiredState),

                    Commands.runOnce( () -> { hook.setDesiredState(HookState.POWERED); } ),
                    Commands.waitUntil( hook::isAtDesiredState),

                    Commands.runOnce( () -> { climber.setDesiredState(ClimberState.BOTTOM); } ),
                    Commands.waitUntil(climber::isAtDesiredState)
                );
    }
    
    public Command retractIntakeCommandSequence() {
        return  Commands.sequence(
                    Commands.runOnce( () -> { intake.setDesiredState(IntakeState.DEPLOYED_OFF); } ),
                    Commands.runOnce( () -> { intake.setDesiredState(IntakeState.INSIDE); } )
            );
    }

    public Command extendIntakeCommandSequence() {
        return  Commands.runOnce( () -> { intake.setDesiredState(IntakeState.DEPLOYED_ON); } );
    }

    public Command runAutoTrajectory(AutoTrajectory trajectory) {
        return  Commands.sequence(
                    trajectory.resetOdometry(),
                    trajectory.cmd(),
                    Commands.waitUntil(trajectory.done())
        );
    }





    public AutoRoutine minMovementShoot() {
        AutoRoutine routine = autoFactory.newRoutine("MinMovementShoot");

        // Load the routine's trajectories
        AutoTrajectory minMovementShootTrajectory = ChoreoTraj.MinMovementShoot.asAutoTraj(routine);

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
                Commands.sequence(
                        minMovementShootTrajectory.resetOdometry(),
                        minMovementShootTrajectory.cmd()));

        // Starting at the event marker named "intake", run the intake
        minMovementShootTrajectory.atTime("Shoot").onTrue( shootPreloadCommandSequence() );

        return routine;
    }

    public AutoRoutine shootAndClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootAndClimb");

        AutoTrajectory shootAndClimb$0 = ChoreoTraj.ShootAndClimb$0.asAutoTraj(routine);
        AutoTrajectory shootAndClimb$1 = ChoreoTraj.ShootAndClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootAndClimb$0),
                runAutoTrajectory(shootAndClimb$1)
            ));

        shootAndClimb$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        
        shootAndClimb$1.atTime("Climb").onTrue( climbCommandSequence() );

        return routine;
    }

    public AutoRoutine shootCollectClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectClimb");

        AutoTrajectory shootCollectClimb$0 = ChoreoTraj.ShootCollectClimb$0.asAutoTraj(routine);
        AutoTrajectory shootCollectClimb$1 = ChoreoTraj.ShootCollectClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootCollectClimb$0),
                runAutoTrajectory(shootCollectClimb$1)
            ));
        
        shootCollectClimb$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        shootCollectClimb$1.atTime("ExtendIntake").onTrue( extendIntakeCommandSequence() );
        shootCollectClimb$1.atTime("RetractIntake").onTrue( retractIntakeCommandSequence() );
        shootCollectClimb$1.atTime("Climb").onTrue( climbCommandSequence() );

        return routine;
    }

    public AutoRoutine shootCollectPass() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectPass");

        AutoTrajectory shootCollectPass$0 = ChoreoTraj.ShootCollectPass$0.asAutoTraj(routine);
        AutoTrajectory shootCollectPass$1 = ChoreoTraj.ShootCollectPass$1.asAutoTraj(routine);
        AutoTrajectory shootCollectPass$2 = ChoreoTraj.ShootCollectPass$2.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootCollectPass$0),
                runAutoTrajectory(shootCollectPass$1),
                runAutoTrajectory(shootCollectPass$2)
        ));

        shootCollectPass$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        shootCollectPass$1.atTime("ExtendIntake").onTrue( extendIntakeCommandSequence() );
        shootCollectPass$1.atTime("Pass").onTrue( shootCollectedFuelCommandSequence() );
        shootCollectPass$2.atTime("Pass").onTrue(
            Commands.sequence(
                shootCollectedFuelCommandSequence(),
                retractIntakeCommandSequence()
        ));

        return routine;
    }

    public AutoRoutine shootCollectShoot() {
        AutoRoutine routine = autoFactory.newRoutine("ShootCollectShoot");

        AutoTrajectory shootCollectShoot$0 = ChoreoTraj.ShootCollectShoot$0.asAutoTraj(routine);
        AutoTrajectory shootCollectShoot$1 = ChoreoTraj.ShootCollectShoot$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootCollectShoot$0),
                runAutoTrajectory(shootCollectShoot$1)
            ));

        shootCollectShoot$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        shootCollectShoot$1.atTime("ExtendIntake").onTrue( extendIntakeCommandSequence() );
        shootCollectShoot$1.atTime("RetractIntake").onTrue( retractIntakeCommandSequence() );
        shootCollectShoot$1.atTime("Shoot").onTrue( shootCollectedFuelCommandSequence() );

        return routine;
    }

    public AutoRoutine shootDepotClimb() {
        AutoRoutine routine = autoFactory.newRoutine("ShootDepotClimb");

        AutoTrajectory shootDepotClimb$0 = ChoreoTraj.ShootDepotClimb$0.asAutoTraj(routine);
        AutoTrajectory shootDepotClimb$1 = ChoreoTraj.ShootDepotClimb$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootDepotClimb$0),
                runAutoTrajectory(shootDepotClimb$1)
            ));
        
        shootDepotClimb$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        shootDepotClimb$1.atTime("ExtendIntake").onTrue( extendIntakeCommandSequence() );
        shootDepotClimb$1.atTime("RetractIntake").onTrue( retractIntakeCommandSequence() );
        shootDepotClimb$1.atTime("Climb").onTrue( climbCommandSequence() );

        return routine;
    }

    public AutoRoutine shootDepotShoot() {
        AutoRoutine routine = autoFactory.newRoutine("ShootDepotShoot");

        AutoTrajectory shootDepotShoot$0 = ChoreoTraj.ShootDepotShoot$0.asAutoTraj(routine);
        AutoTrajectory shootDepotShoot$1 = ChoreoTraj.ShootDepotShoot$1.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                runAutoTrajectory(shootDepotShoot$0),
                runAutoTrajectory(shootDepotShoot$1)
            ));

        shootDepotShoot$0.atTime("Shoot").onTrue( shootPreloadCommandSequence() );
        shootDepotShoot$1.atTime("ExtendIntake").onTrue( extendIntakeCommandSequence() );
        shootDepotShoot$1.atTime("RetractIntake").onTrue( retractIntakeCommandSequence() );
        shootDepotShoot$1.atTime("Shoot").onTrue( shootCollectedFuelCommandSequence() );

        return routine;
    }
}
