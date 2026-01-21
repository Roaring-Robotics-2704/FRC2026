package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperStructureStates.WantedState;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.hopper.Hopper.HopperState;

/**
 * SuperStructure subsystem for controlling the robot's superstructure
 * mechanisms.
 */
public class SuperStructure extends SubsystemBase {
    private WantedState wantedState = WantedState.IDLE;
    private WantedState currentState = WantedState.IDLE;

    // Subsystems
    private final Hopper hopper;

    /** Creates a new SuperStructure. */
    public SuperStructure(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    public void periodic() {
        if (currentState == WantedState.TRANSITIONING) {
            // Here would be the logic to check if the superstructure has reached the wanted
            // state
            // For now, we will assume it reaches the wanted state immediately for
            // simplicity
            switch (wantedState) {
                case START:
                    hopper.setDesiredState(HopperState.IDLE);
                    break;
                case IDLE:
                    hopper.setDesiredState(HopperState.IDLE);
                    break;
                case INTAKE:
                    hopper.setDesiredState(HopperState.REVERSING);
                    break;
                case PASS:
                    hopper.setDesiredState(HopperState.FEEDING);
                    break;
                case FEED:
                    hopper.setDesiredState(HopperState.FEEDING);
                    break;
                case SHOOT:
                    hopper.setDesiredState(HopperState.FEEDING);
                    break;
                case CLIMB:
                    hopper.setDesiredState(HopperState.IDLE);
                    break;
                // Add additional cases as needed
                default:
                    throw new IllegalStateException("Unexpected value: " + wantedState);
            }
            if (hopper.isAtWantedState()) {
                currentState = wantedState;
            }
        }
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
     *            the desired WantedState to transition to
     */
    public void setWantedState(WantedState state) {
        // Only change if different than what is it at, or what it is going to
        if (state == WantedState.TRANSITIONING) {
            throw new IllegalArgumentException(
                    "Cannot set wanted state to TRANSITIONING directly.");
        } else if (state != currentState || state != wantedState) {
            this.wantedState = state;
            currentState = WantedState.TRANSITIONING;
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
    public Command goToState(WantedState state) {
        return Commands.sequence(
                Commands.runOnce(() -> setWantedState(state), this),
                Commands.waitUntil(this::isAtWantedState)).finallyDo(() -> setWantedState(WantedState.IDLE));
    }

    public WantedState getWantedState() {
        return this.wantedState;
    }

    public void setCurrentState(WantedState state) {
        this.currentState = state;
    }

    public WantedState getCurrentState() {
        return this.currentState;
    }

    public boolean isAtWantedState() {
        return this.currentState == this.wantedState;
    }
}
