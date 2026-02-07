package frc.robot;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Field-related constants for the game. All measurements are by default relative to the blue alliance.
 */
public class FieldConstants {
    /** The field length in meters. */
    public static final double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
    /** The field width in meters. */
    public static final double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
}