package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

public interface ClimberIO {


    @AutoLog
    class ClimberIOInputs {
        public int climberHolePosition = 0;
        public MutVoltage climberAppliedVolts = Volts.mutable(0);
        public MutCurrent climberCurrentAmps = Amps.mutable(0);
    }

    default void updateInputs(ClimberIOInputs inputs) {}
    default void runVolts (Voltage volts){}
     ;
     void stop(){};
     



}
