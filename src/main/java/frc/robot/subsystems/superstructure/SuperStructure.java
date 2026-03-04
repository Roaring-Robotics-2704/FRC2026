package frc.robot.subsystems.superstructure;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperStructureConstants.SuperStructureStates;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.climber.Climber.ClimberState;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.hopper.Hopper.HopperState;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.intake.Intake.IntakeState;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.shooter.Shooter.ShooterState;

/**
 * SuperStructure subsystem for controlling the robot's superstructure
 * mechanisms.
 */
public class SuperStructure extends SubsystemBase {
    private SuperStructureStates wantedState = SuperStructureStates.IDLE;
    private SuperStructureStates currentState = SuperStructureStates.IDLE;

    // Subsystems
    private final Intake intake;
    private final Hopper hopper;
    private final Kicker kicker;
    private final Shooter shooter;
    private final Climber climber;

    /** Creates a new SuperStructure. */
    public SuperStructure(Intake intake, Hopper hopper, Kicker kicker, Shooter shooter, Climber climber) {
        this.hopper = hopper;
        this.kicker = kicker;
        this.shooter = shooter;
        this.intake = intake;
        this.climber = climber;
    }

    @Override
    public void periodic() {
        if (currentState != wantedState) {
            switch (wantedState) {
                case START:
                    intake.setDesiredState(IntakeState.INSIDE);
                    hopper.setDesiredState(HopperState.IDLE);
                    kicker.setKickerVoltage(-1);
                    shooter.setDesiredState(ShooterState.IDLE);
                    break;
                case IDLE:
                    kicker.setKickerVoltage(-1);
                    intake.setDesiredState(IntakeState.STOWED);
                    hopper.setDesiredState(HopperState.IDLE);
                    shooter.setDesiredState(ShooterState.IDLE);
                    break;
                case INTAKE:
                    kicker.setKickerVoltage(-1);
                    intake.setDesiredState(IntakeState.DEPLOYED_ON);
                    hopper.setDesiredState(HopperState.IDLE);
                    // shooter.setDesiredState(ShooterState.IDLE);
                    break;
                case PASS:
                    kicker.setKickerVoltage(12);
                    intake.setDesiredState(IntakeState.STOWED);
                    hopper.setDesiredState(HopperState.FEEDING);
                    shooter.setDesiredState(ShooterState.SHOOTING);
                    break;
                case FEED:
                    kicker.setKickerVoltage(12);
                    intake.setDesiredState(IntakeState.DEPLOYED_ON);
                    hopper.setDesiredState(HopperState.FEEDING);
                    shooter.setDesiredState(ShooterState.SHOOTING);
                    break;
                case SHOOTER_PREP:
                    kicker.setKickerVoltage(-1);
                    hopper.setDesiredState(HopperState.IDLE);
                    shooter.setDesiredState(ShooterState.SHOOTING);
                    break;
                case SHOOT:
                    kicker.setKickerVoltage(12);
                    intake.setDesiredState(IntakeState.STOWED);
                    hopper.setDesiredState(HopperState.FEEDING);
                    shooter.setDesiredState(ShooterState.SHOOTING);
                    break;
                case CLIMB_PREP:
                    kicker.setKickerVoltage(-1);
                    intake.setDesiredState(IntakeState.INSIDE);
                    hopper.setDesiredState(HopperState.IDLE);
                    shooter.setDesiredState(ShooterState.STATIONARY);
                    climber.setDesiredState(ClimberState.TOP);
                    break;
                case CLIMB:
                    kicker.setKickerVoltage(-1);
                    intake.setDesiredState(IntakeState.INSIDE);
                    hopper.setDesiredState(HopperState.IDLE);
                    shooter.setDesiredState(ShooterState.STATIONARY);
                    climber.setDesiredState(ClimberState.MIDDLE);
                    break;
                case INTAKE_CALIBRATE_IN:
                    intake.setDesiredState(IntakeState.CALIBRATE_IN);
                    break;
                case INTAKE_CALIBRATE_OUT:
                    intake.setDesiredState(IntakeState.CALIBRATE_OUT);
                    break;
                // Add additional cases as needed
                default:
                    throw new IllegalStateException("Unexpected value: " + wantedState);
            }
            // if (hopper.isAtDesiredState() && shooter.isAtDesiredState()
            //         && intake.atDesiredState() && climber.isAtDesiredState()) {
            //     currentState = wantedState;
            // }
            if (hopper.isAtDesiredState() && shooter.isAtDesiredState() && intake.atDesiredState()) {
                currentState = wantedState;
                
            }
        }
        Logger.recordOutput("SuperStructure/Current State", currentState.toString());
        Logger.recordOutput("SuperStructure/Wanted State", wantedState.toString());
    }

    /**
     * Request a change to the superstructure's desired state.
     *
     * <p>If the current state differs from the currently stored wanted state, this
     * method
     * marks the current state as TRANSITIONING to indicate that a change is in
     * progress.
     * The supplied state is then recorded as the new wanted state, which the
     * subsystem's
     * periodic/control logic will act upon to reach the requested configuration.
     *
     * @param state
     *            the desired SuperStructureStates to transition to
     */
    public void setDesiredState(SuperStructureStates state) {
        // Only change if different than what is it at, or what it is going to
        if (state != currentState || state != wantedState) {
            this.wantedState = state;
        }
    }

    /**
     * Command to transition the superstructure to the specified wanted state.
     *
     * <p>This command first sets the desired wanted state using setWantedState(state),
     * then waits until the superstructure reports that it has reached the wanted
     * state
     * via isAtWantedState(). This ensures that the command only completes once the
     * superstructure has fully transitioned to the requested configuration.
     *
     * @param state
     *            the WantedState to transition to
     * @return a Command that performs the state transition
     */
    public Command goToState(SuperStructureStates state) {
        return Commands.sequence(
                Commands.runOnce(() -> setDesiredState(state), this));
    }

    public Command goToStateWithIdle(SuperStructureStates state) {
        return Commands.sequence(
                Commands.runOnce(() -> setDesiredState(state), this),
                Commands.waitUntil(this::isAtDesiredState)).finallyDo(() -> setDesiredState(SuperStructureStates.IDLE));
    }

    public SuperStructureStates getWantedState() {
        return this.wantedState;
    }

    public void setCurrentState(SuperStructureStates state) {
        this.currentState = state;
    }

    public SuperStructureStates getCurrentState() {
        return this.currentState;
    }

    public boolean isAtDesiredState() {
        return this.currentState == this.wantedState;
    }

    // public Command driveCommand(
    //         Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    //     return Commands.either(
    //             DriveCommands.joystickDrive(drive, xSupplier, ySupplier, omegaSupplier),
    //             DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, shooter::getWantedRobotAngle),
    //             () -> wantedState == SuperStructureStates.SHOOT);
    // }
}
