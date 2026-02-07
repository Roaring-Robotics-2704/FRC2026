// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.example;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/**
 * IO base for the example mechanism.
 * This class also acts as the default "replay" implementation.
 *
 * @see ExampleMech
 *      Allows swapping between real and simulated IO, along with replay
 */
public interface ExampleMechIO {

    /** Inputs read from the intake hardware.
     *Use Mutable types when possible for easy updating and thread safety, along with reducing garbage creation
     */
    @AutoLog // Annotation for automatic logging with AdvantageKit
    public static class ExampleMechIOInputs {
        MutVoltage appliedVoltage = Volts.mutable(0);
        MutCurrent currentDraw = Amps.mutable(0);
        boolean endstopHit = false;
        MutAngularVelocity rollerVelocity = RotationsPerSecond.mutable(0);
    }

    /** Updates the inputs structure with the latest values from hardware. */
    default void updateInputs(ExampleMechIOInputs inputs) {
        // Default to nothing for replay
        // Replay will fill values from log file
    }

    /**EXAMPLE Sets the brake mode on the motors for the mechanism. */
    default void setBrakeMode(boolean enabled) {
        // Default to nothing for replay
        // Replay will fill values from log file
    }

    /**EXAMPLE Sets the voltage for the mechanism motors. */
    default void setVoltage(Voltage voltage) {
        // Default to nothing for replay
        // Replay will fill values from log file
    }
}