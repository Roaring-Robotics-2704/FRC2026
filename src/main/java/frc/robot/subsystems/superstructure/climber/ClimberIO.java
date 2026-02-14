package frc.robot.subsystems.superstructure.climber;

import org.littletonrobotics.junction.AutoLog;

/** The IO for the climber subsystem. */
public interface ClimberIO {

    /** Inputs for the climber subsystem. */
    @AutoLog
    public static class ClimberIOInputs {

    }

    default void updateInputs(ClimberIOInputs inputs) {
        // Default to nothing for replay
    }

}
