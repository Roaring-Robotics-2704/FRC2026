// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class HopperConstants {
    public static final Voltage HOPPER_VOLTAGE = Volts.of(6);
    // Convenience numeric representation (volts) used by SlewRateLimiter and calculations
    public static final int HOPPER_CURRENT_LIMIT = 20;
    public static final int HOPPER_MOTOR_ID = 30;
    public static final double HOPPER_VOLTAGE_RAMP_RATE = 0.1; // Volt change per cycle (20 hz)
}
