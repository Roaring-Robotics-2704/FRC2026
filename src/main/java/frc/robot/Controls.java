package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Kicker.Kicker;
import frc.robot.subsystems.superstructure.climber.Climber;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OrchestraManager;
import frc.robot.util.simUtils.Simulation;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Controls {
    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)", AlertType.kWarning);
    private final Alert coDriverDisconnectedAlert = new Alert("Co-driver controller disconnected (port 1)", AlertType.kInfo);

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController coDriver = new CommandXboxController(1);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 20.0);

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
    }

    /** Configures the controls. */
    public void configureControls(RobotContainer rc) {
        Drive drive = rc.drive;
        Hopper hopper = rc.hopper;
        Kicker kicker = rc.kicker;
        Climber climber = rc.climber;
        Intake intake = rc.intake;
        Shooter shooter = rc.shooter;
        Vision vision = rc.vision;
        
        
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));


        vision.setDefaultCommand(vision.idle());
        // objectDetection.setDefaultCommand(objectDetection.idle());



        // Reset gyro to 0 deg when B button is pressed
        driver.leftTrigger().whileTrue(Commands.parallel(
            Commands.startEnd(
                () -> intake.setDesiredState(Intake.IntakeState.DEPLOYED_ON),
                () -> intake.setDesiredState(Intake.IntakeState.STOWED), intake
                )
        ));
        driver.rightTrigger().whileTrue(Commands.parallel(
            Commands.startEnd(()->{
            if (shooter.isAtDesiredState())
            {kicker.setKickerVoltage(12);}}
            ,()->kicker.setKickerVoltage(-1), kicker),
            Commands.startEnd(()->shooter.setDesiredState(Shooter.ShooterState.SHOOTING), ()->shooter.setDesiredState(Shooter.ShooterState.IDLE), shooter),
            Commands.startEnd(()->hopper.setDesiredState(Hopper.HopperState.FEEDING), ()->hopper.setDesiredState(Hopper.HopperState.IDLE), hopper)
        ));

        driver.y().onTrue(OrchestraManager.getInstance().playOrchestraCommand("thx").ignoringDisable(true));

        
        // Reset gyro to 0 deg when B button is pressed

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.isSim ? () -> drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(RobotState.getInstance().getPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero
                    : Rotation2d.k180deg)); // Zero gyro
        final Runnable resetOdometry = Constants.isSim
            ? () -> drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(
                new Pose2d(0, 0, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        driver.start().and(driver.leftStick()).debounce(0.5).onTrue(Commands.runOnce(resetOdometry, drive).ignoringDisable(true));
        
        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(RumbleType.kBothRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(RumbleType.kBothRumble).withTimeout(0.4).andThen(Commands.waitSeconds(0.3)).repeatedly().withTimeout(2.0));
    }

    private HashMap<Integer, Double> driverRumbleCommands = new HashMap<>();

    public void setDriverRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            driverRumbleCommands.remove(hash);
        } else {
            driverRumbleCommands.put(hash, value);
        }
        driver.setRumble(type, driverRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public Command controllerRumbleWhileRunning(RumbleType type) {
        return Commands.startEnd(() -> {
            setDriverRumble(type, 1.0, hashCode());
        }, () -> {
            setDriverRumble(type, 0.0, hashCode());
        }).withName("ControllerRumbleWhileRunning");
    }

    /** Updates the controls, including shown alerts. */
    public void update() {
        // Controller disconnected alerts
        int driverPort = driver.getHID().getPort();
        int coDriverPort = coDriver.getHID().getPort();
        driverDisconnectedAlert.set(!DriverStation.isJoystickConnected(driverPort) || !DriverStation.getJoystickIsXbox(driverPort));
        coDriverDisconnectedAlert.set(!DriverStation.isJoystickConnected(coDriverPort) || !DriverStation.getJoystickIsXbox(coDriverPort));
    }

}
