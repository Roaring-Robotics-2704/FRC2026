// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

    @AutoLog
    public class ClimberIOInputs {
        public MutDistance climberPosition = Inches.mutable(0);
        public MutCurrent climberCurrent = Amps.mutable(0);
        public MutLinearVelocity climberVelocity = InchesPerSecond.mutable(0);
        public MutVoltage climberVoltage = Volts.mutable(0);

        public MutCurrent hookCurrent = Amps.mutable(0);
    }

    default void setClimberVoltage(double voltage) {
    }

    default void resetClimberEncoder() {
    }

    default void setHookVoltage(double voltage) {
    }

    default void updateInputs(ClimberIOInputs inputs) {
    }
}
