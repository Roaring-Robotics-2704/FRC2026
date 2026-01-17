// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

/** Hopper  */
public class HopperIO {
    public class HopperIOInputs {
        public MutCurrent currentDraw = Amps.mutable(0);
        public MutVoltage appliedVoltage = Volts.mutable(0);
        public MutAngularVelocity motorVelocity = RotationsPerSecond.mutable(0);
    }
}
