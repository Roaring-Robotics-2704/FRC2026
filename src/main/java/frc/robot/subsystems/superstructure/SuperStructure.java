package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperStructureStates.CurrentState;
import frc.robot.subsystems.superstructure.SuperStructureStates.WantedState;
import frc.robot.subsystems.superstructure.hopper.Hopper;

/** SuperStructure subsystem for controlling the robot's superstructure mechanisms. */
public class SuperStructure extends SubsystemBase {

    public SuperStructure(Hopper hopper) {
    }

    @Override
    public void periodic() {

    }

    private WantedState wantedState = WantedState.IDLE;
    private CurrentState currentState = CurrentState.IDLE;

 
    /**
     * Request a change to the superstructure's desired state.
     *
     * <p>If the current state differs from the currently stored wanted state, this method
     * marks the current state as TRANSITIONING to indicate that a change is in progress.
     * The supplied state is then recorded as the new wanted state, which the subsystem's
     * periodic/control logic will act upon to reach the requested configuration.
     *
     * @param state the desired WantedState to transition to
     */
    public void setWantedState(WantedState state) {
        if (currentState.toString() != wantedState.toString()) {
            currentState = CurrentState.TRANSITIONING;
        }
        this.wantedState = state;
    }

    public WantedState getWantedState() {
        return this.wantedState;
    }

    public void setCurrentState(CurrentState state) {
        this.currentState = state;
    }

    public CurrentState getCurrentState() {
        return this.currentState;
    }
}
