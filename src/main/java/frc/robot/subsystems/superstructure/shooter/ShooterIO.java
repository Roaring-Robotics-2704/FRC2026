// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

  /** Creates a new ShooterIO. */
  public interface ShooterIO { // creates and sets baseline values of robot for logging
    @AutoLog
    class ShooterIOInputs {
      public double ShooterAngle = 0.0;
      public double ShooterVelocity = 0.0;
      public double shootShooterAppliedVolts = 0.0;
      public double angleShooterAppliedVolts = 0.0;
      public double shootShooterCurrentAmps = 0.0;
      public double angleShooterCurrentAmps = 0.0;
    }
  

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVelocity(double velocityRadperSec) {}

  default void runVolts(Voltage volts) {};

  default void runSetpoint(TrapezoidProfile.State position) {};

  default void stop() {};

  default void init() {};
}
