// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/**
 * IO implementation for the intake.
 *
 * @see Intake
 *      Allows swapping between real and simulated IO, along with replay
 */
public interface IntakeIO {
    /** Inputs read from the intake hardware. */
    @AutoLog
    public static class IntakeIOInputs {
        // Slide values
        public MutVoltage slideAppliedVoltage = Volts.mutable(0);
        public MutCurrent slideCurrentDraw = Amps.mutable(0);
        public MutDistance slidePosition = Inches.mutable(0);
        public MutLinearVelocity slideVelocity = InchesPerSecond.mutable(0);
        public boolean slideAtPosition = false;

        // Roller values
        public MutVoltage rollerAppliedVoltage = Volts.mutable(0);
        public MutCurrent rollerCurrentDraw = Amps.mutable(0);
        public MutAngularVelocity rollerVelocity = RotationsPerSecond.mutable(0);
    }

    default void updateInputs(IntakeIOInputs inputs) {
        // Default to nothing for replay
    }


    default void setSlideVoltage(Voltage voltage) {
        // Default to nothing for replay
    }

    default void setRollerVoltage(Voltage voltage) {
        // Default to nothing for replay
    }

    default void setPosition(Distance position) {
        // Default to nothing for replay
    }

    default void stopMotors() {
        // Default to nothing for replay
    }

    default void resetSlideEncoder(Distance position) {
        // Default to nothing for replay
    }

}
