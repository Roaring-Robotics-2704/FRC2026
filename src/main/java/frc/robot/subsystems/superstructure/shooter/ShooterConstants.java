// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import com.revrobotics.AnalogInput;

  /** Creates a new ShooterConstants. */
  public class ShooterConstants {
    private ShooterConstants() {}

    public static final double MAX_SHOOTER_VOLTAGE = 10; // max # of voltage in shooter, change dummy value later

    public static final int SHOOTER_MOTOR_ONE = 998; // sets motor CAN ID, this is a dummy motor id, change this later
    public static final int SHOOTER_MOTOR_TWO = 999; // sets motor CAN ID, this is a dummy motor id, change this later

    public static final int SHOOTER_ENCODER_ONE = 998; // sets encoder ID, this is a dummy encoder #, change this later
    public static final int SHOOTER_ENCODER_TWO = 999; // sets encoder ID, this is a dummy encoder #, change this later

    public static final int CURRENT_LIMIT = 10; // shooter speed limit, this is a dummy value, change this later
    public static final double GEAR_REDUCTION = 3 / 1; // this is a dummy value, change this later, might not be necessary
    public static final DCMotor SHOOTER_MOTOR_TYPE = DCMotor.getNEO(2); // change this later if necessary

    public static final double CARRIAGE_MASS = Units.lbsToKilograms(5); // change this later if necessary

    public static final double SHOOTER_KP = 1.0; // more error = more power
    public static final double SHOOTER_KI = 0; // accumulates error
    public static final double SHOOTER_KD = 0.0; // predicts ROC of error, change these PID values later as needed
    public static final double SHOOTER_KF = 0.0; // predicts power needed to maintain a specific state

    // public static final double SHOOTER_KTARGETRPM = 0.0; // add these if we need them, desired speed
    // public static final double SHOOTER_KTOLERANCERPM = 0.0; // add these if we need them, at speed

    // GET REAL VALUES FOR FOLLOWING:
    public static final double MIN_ANGLE = Units.inchesToMeters(0); // lowest angle shooter can reach
    public static final double MAX_ANGLE = Units.inchesToMeters(5); // largest angle shooter can reach
  }

