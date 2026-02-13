package frc.robot.subsystems.superstructure.climber;

public class ClimberConstants {
    // canID
    public static final int TELESCOPE_MOTOR_ID = 100;
    public static final int HOOK_MOTOR_ID = 101;

    public static final DCMotor HOOK_MOTOR_TYPE = DCMotor.get
    public static final DCMotor TELESCOPE_MOTOR_TYPE = 

    //
    public static final double HOOK_TARGET_DEGREES = 180;

    //Climber heights
    public static final double RETRACTED_HEIGHT = 20.5;
    public static final double EXTENDED_HEIGHT = 29.5;

    //Speed of climber arm and hook
    public static final double HOOK_SPEED = .4;
    public static final double TELESCOPE_SPEED = .6;

    public static final int TELESCOPE_CURRENT_LIMIT = 20;
    public static final int HOOK_CURRENT_LIMIT = 20;
}
