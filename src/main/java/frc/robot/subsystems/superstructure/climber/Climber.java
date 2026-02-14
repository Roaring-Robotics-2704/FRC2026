package frc.robot.subsystems.superstructure.climber;
import  edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

    private final ClimberIO io;

    private ClimberState currentState = ClimberState.INSIDE;
    private ClimberState desiredState = ClimberState.INSIDE;

    public Climber(ClimberIO climberIO) {
        this.io = climberIO;
    }


    public void setDesiredState(ClimberState state) {
        this.desiredState = state;
    }

    public boolean isAtDesiredState() {
        return currentState == desiredState;
    }


    public enum ClimberState {
        INSIDE,
        OUTSIDE_UP,
        OUTSIDE_DOWN
    }

    

}
