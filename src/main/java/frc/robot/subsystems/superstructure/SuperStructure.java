package frc.robot.subsystems.superstructure;
import static frc.robot.subsystems.superstructure.SuperStructureStates.*;

public class SuperStructure {

    private SuperStructure() {
    }

    private static SuperStructure instance;
    public static SuperStructure getInstance() {
        if (instance == null) {
            instance = new SuperStructure();
        }
        return instance;
    }
    private WantedState wantedState = WantedState.IDLE;
    private CurrentState currentState = CurrentState.IDLE;

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
