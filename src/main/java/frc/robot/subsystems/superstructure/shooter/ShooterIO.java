// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** Creates a new ShooterIO. */
public interface ShooterIO { // creates and sets baseline values of robot for logging
    /** Inputs for the ShooterIO. */
    @AutoLog
    public static class ShooterIOInputs {
        public MutAngle hoodAngle = Degrees.mutable(0);
        public MutAngularVelocity flywheelVelocity = DegreesPerSecond.mutable(0);
        public MutAngularAcceleration flywheelAcceleration = DegreesPerSecondPerSecond.mutable(0);
        public MutVoltage flywheelAppliedVolts = Volt.mutable(0);
        public MutCurrent flywheelCurrentAmps = Amps.mutable(0);

        public boolean atTargetVelocity = false;
        public boolean atTargetAngle = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    // voltage methods for flywheel and hood
    default void setFlywheelVoltage(Voltage voltage) {
    }
    
    default void setHoodVoltage(Voltage voltage) {
    }

    default void setHoodAngle(Angle angle) {
    }

    default void setFlywheelVelocity(AngularVelocity velocity) {
    }

    

}
