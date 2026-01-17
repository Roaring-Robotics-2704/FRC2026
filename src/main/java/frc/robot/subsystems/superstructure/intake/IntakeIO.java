// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** IO implementation for the intake.
    *
    *     @see Intake
    *     Allows swapping between real and simulated IO, along with replay
    */
public interface IntakeIO {
  /** Inputs read from the intake hardware. */
  public class IntakeIOInputs {
    // Slide values
    MutVoltage slideAppliedVoltage = Volts.mutable(0);
    MutCurrent slideCurrentDraw = Amps.mutable(0);
    AtomicBoolean slideEndstopHit = new AtomicBoolean(false);

    // Roller values
    MutVoltage rollerAppliedVoltage = Volts.mutable(0);
    MutCurrent rollerCurrentDraw = Amps.mutable(0);
    MutAngularVelocity rollerVelocity = RotationsPerSecond.mutable(0);
  }

  default void updateInputs(IntakeIOInputs inputs) {
    // Default to nothing for replay
  }

  default void setSlideBrakeMode(boolean enabled) {
    // Default to nothing for replay
  }

  default void setSlideVoltage(Voltage voltage) {
    // Default to nothing for replay
  }

  default void setRollerVoltage(Voltage voltage) {
    // Default to nothing for replay
  }
  


}
