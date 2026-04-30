package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import static edu.wpi.first.units.Units.Degrees;

public class VisionConstants {
    /****** Simulation ******/
    /**
     * If we should enable vision simulation. Turning off vision sim can dramatically improve loop times, but it's
     * obviously far less representative of real robot odometry.
     */
    public static final boolean enableVisionSimulation = false;

    /**
     * Enable drawing a wireframe visualization of the field to the camera streams in simulation mode. This is extremely
     * resource-intensive!
     */
    public static final boolean enableWireframeDrawing = false;

    /**
     * Enable raw streams for simulated cameras. This can increase loop times slightly.
     */
    public static final boolean enableRawStreams = true;
    /************************/

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "Shooter Cam";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(-0.171450, -0.061, 0.487, new Rotation3d(Degrees.of(0), Degrees.of(-25.378547), Degrees.of(90)));

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;
    /** The maximum error in an estimate's rotation in degrees. */
    public static final double maxRotationError = 180;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
