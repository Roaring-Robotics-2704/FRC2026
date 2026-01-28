// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import com.revrobotics.AnalogInput;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

  /** Creates a new ShooterConstants. */
  public class ShooterConstants {
    private ShooterConstants() {}

    public static final Voltage MAX_SHOOTER_VOLTAGE = Volts.of(12); // max # of voltage in shooter, change dummy value later

    public static final int FLYWHEEL_MOTOR_ONE = 998; // sets motor CAN ID, this is a dummy motor id, change this later
    public static final int FLYWHEEL_MOTOR_TWO = 999; // sets motor CAN ID, this is a dummy motor id, change this later

    public static final int HOOD_SERVO1_PORT = 1; // sets servo port #, this is a dummy encoder #, change this later
    public static final int HOOD_SERVO2_PORT = 2; // sets servo port #, this is a dummy encoder #, change this later

    public static final int CURRENT_LIMIT = 40; // shooter speed limit, this is a dummy value, change this later
    public static final DCMotor SHOOTER_MOTOR_TYPE = DCMotor.getKrakenX60(2); // change this later if necessary

    public static final double SHOOTER_KP = 1.0; // more error = more power
    public static final double SHOOTER_KI = 0; // accumulates error
    public static final double SHOOTER_KD = 0.0; // predicts ROC of error, change these PID values later as needed
    public static final double SHOOTER_KV = 0.0; 
    public static final double SHOOTER_KA = 0.0;
    public static final double SHOOTER_KS = 0.0;

    public static final AngularVelocity SHOOTER_TARGET_RPM = RotationsPerSecond.of(2800); // add these if we need them, desired speed
    public static final AngularVelocity SHOOTER_IDLE_RPM = RotationsPerSecond.of(1000); // add these if we need them, idle speed
    public static final double SHOOTER_TOLERANCE_RPM = 0.0; // add these if we need them, at speed
    
    // GET REAL VALUES FOR FOLLOWING:
    public static final Angle MIN_ANGLE = Degrees.of(0); // lowest angle shooter can reach
    public static final Angle MAX_ANGLE = Degrees.of(45); // largest angle shooter can reach
  }

