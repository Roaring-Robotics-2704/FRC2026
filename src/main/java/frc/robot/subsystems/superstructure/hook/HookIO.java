// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hook;

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
 * @see HookIO
 *      Allows swapping between real and simulated IO, along with replay
 */
public interface HookIO {

    /** Inputs read from the intake hardware.
     *Use Mutable types when possible for easy updating and thread safety, along with reducing garbage creation
     */
    @AutoLog
    public static class HookIOInputs {
        MutVoltage hookVoltage = Volts.mutable(0);
        MutCurrent hookCurrent = Amps.mutable(0);
    }

    default void updateInputs(HookIOInputs inputs) {
    }

    default void setMotorVoltage(Voltage voltage) {
    }
}