package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Small helper to expose an auto chooser on Shuffleboard and map selections to Commands.
 *
 * Usage:
 * - Instantiate in RobotContainer and call `publish()` to put the chooser on Shuffleboard.
 * - Call `getSelectedCommand()` from Robot.autonomousInit() to get a Command to schedule.
 *
 * NOTE: Adapt create*Command methods to match your ChoreoTraj / PathPlanner command constructors.
 */
public class AutoSelector {
    private final SendableChooser<String> chooser = new SendableChooser<>();

    public AutoSelector() {
        // default option
        chooser.setDefaultOption("Do Nothing", "Do Nothing");

        // trajectory files found in src/main/deploy/choreo/
        chooser.addOption("Shoot And Climb", "ShootAndClimb.traj");
        chooser.addOption("Collect Depo Climb", "CollectDepoClimb.traj");
        chooser.addOption("Min Movement Shoot", "MinMovementShoot.traj");

        // Add any other autos here. Use friendly names for display and stable ids for mapping.
    }

    /** Publish the chooser to Shuffleboard/SmartDashboard. Call from RobotContainer constructor. */
    public void publish() {
        SmartDashboard.putData("Auto Mode", chooser);
    }

    /**
     * Return a Command for the currently selected auto. If selection is Do Nothing or unknown,
     * returns an InstantCommand no-op.
     *
     * IMPORTANT: The returned Command should be a freshly created command so it only runs when
     * scheduled from autonomousInit.
     */
    public Command getSelectedCommand() {
        String selected = chooser.getSelected();
        if (selected == null || selected.equals("Do Nothing")) {
            return new InstantCommand();
        }

        switch (selected) {
            case "ShootAndClimb.traj":
                return createChoreoCommand("ShootAndClimb.traj");
            case "CollectDepoClimb.traj":
                return createChoreoCommand("CollectDepoClimb.traj");
            case "MinMovementShoot.traj":
                return createChoreoCommand("MinMovementShoot.traj");
            default:
                return new InstantCommand();
        }
    }

    // ====== Helper factory methods - adapt these to your project's command constructors ======

    private Command createChoreoCommand(String trajFile) {
        // Placeholder implementation. Replace with your real ChoreoTraj usage.
        // Example if ChoreoTraj is a Command: return new ChoreoTraj(trajFile, ...);

        // For safety we return a no-op here so this file compiles without referencing
        // other project classes. The user should edit this method to return the correct command.
        return new InstantCommand();

        // Example replacement:
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> drive.resetPose(...), drive),
        //     new ChoreoTraj(trajFile, drive, intake, superstructure)
        // );
    }
}
