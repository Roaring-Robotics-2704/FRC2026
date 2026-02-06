// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class IntakeConstants {
    public static final int ROLLER_MOTOR_ID = 10;
    public static final int SLIDE_MOTOR_ID = 11;

    public static final int ROLLER_CURRENT_LIMIT = 20;
    public static final int SLIDE_CURRENT_LIMIT = 20;

    public static final Distance SLIDE_MAX_DISTANCE = Inches.mutable(12);

    public static final double SLIDE_POSITION_KP = 5.0;
    public static final double SLIDE_POSITION_KI = 0.0;
    public static final double SLIDE_POSITION_KD = 0.0;
    public static final double SLIDE_POSITION_KS = 0.2;
    public static final double SLIDE_POSITION_KV = 0.0;
    public static final double SLIDE_POSITION_KG = 0.0;
    public static final double SLIDE_POSITION_KA = 0.0;

    public static final double SLIDE_MAX_ACCELERATION = 1.0;
    public static final double SLIDE_MAX_VELOCITY = 1.0;
    public static final double ENCODER_GEAR_RATIO = 1; //TODO: Update this value


    public static final Distance SLIDE_POSITION_TOLERANCE = Inches.of(0.1);

}
