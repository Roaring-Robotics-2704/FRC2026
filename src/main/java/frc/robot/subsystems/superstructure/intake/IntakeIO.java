// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import java.util.concurrent.atomic.AtomicBoolean;

/** Add your docs here. */
public class IntakeIO {
  public class IntakeIOInputs {
    MutVoltage appliedVoltage = Volts.mutable(0);
    MutCurrent currentDraw = Amps.mutable(0);
    AtomicBoolean endstopHit = new AtomicBoolean(false);
  }
}
