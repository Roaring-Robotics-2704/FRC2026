// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

/** Creates a new ShooterConstants. */
public class ShooterConstants {
    private ShooterConstants() {}
    
    public static final Voltage MAX_SHOOTER_VOLTAGE = Volts.of(12); // max # of voltage in shooter, change dummy value later//TODO does this need to be changed 

    public static final int FLYWHEEL_MOTOR_ONE = 22; // sets motor CAN ID, this is a dummy motor id, change this later
    public static final int FLYWHEEL_MOTOR_TWO = 23; // sets motor CAN ID, this is a dummy motor id, change this later

    public static final int HOOD_SERVO1_PORT = 9; // sets servo port #, this is a dummy encoder #, change this later

    public static final int CURRENT_LIMIT = 40; // shooter speed limit, this is a dummy value, change this later
    public static final DCMotor SHOOTER_MOTOR_TYPE = DCMotor.getKrakenX60(2); // change this later if necessary

    public static final double SHOOTER_KP = 0.0; // more error = more power
    public static final double SHOOTER_KD = 0.0; // predicts ROC of error, change these PID values later as needed
    public static final double SHOOTER_KV = 0.1; 
    public static final double SHOOTER_KA = 30.0;
    public static final double SHOOTER_KS = 0.0;

    public static final AngularVelocity SHOOTER_TARGET = 
        RPM.of(2800); // add these if we need them, desired speed
    public static final AngularVelocity SHOOTER_IDLE = 
        RPM.of(2000); // add these if we need them, idle speed
    public static final AngularVelocity SHOOTER_TOLERANCE = RotationsPerSecond.of(50);    
    // GET REAL VALUES FOR FOLLOWING:
    public static final Angle MIN_ANGLE = Degrees.of(28.048335); // lowest angle shooter can reach
    public static final Angle MAX_ANGLE = Degrees.of(59.874070); // largest angle shooter can reach
    
}

