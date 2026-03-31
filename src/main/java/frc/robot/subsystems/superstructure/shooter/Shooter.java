// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MAX_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_IDLE;

import frc.robot.util.solvers.BasicTunedCalc;
import frc.robot.util.solvers.BasicTunedCalc.ShootingSolution;
import frc.robot.util.tunables.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** The shooter subsystem. */
public class Shooter extends SubsystemBase {

    private static ShooterState currentState = ShooterState.STATIONARY;
    private static ShooterState desiredState = ShooterState.STATIONARY;

    private double hoodPercent;
    private AngularVelocity flywheelVelocity;
    private final BasicTunedCalc solver = new BasicTunedCalc();

    private final ShooterIO io;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunableNumber shootP = new LoggedTunableNumber("Shooter/ShootP");
    private static final LoggedTunableNumber shootD = new LoggedTunableNumber("Shooter/ShootD");

    private static final LoggedTunableNumber shootS = new LoggedTunableNumber("Shooter/ShootS");
    private static final LoggedTunableNumber shootV = new LoggedTunableNumber("Shooter/ShootV");
    private static final LoggedTunableNumber shootA = new LoggedTunableNumber("Shooter/ShootA");
    private AngularVelocity target = ShooterConstants.SHOOTER_TARGET;
    private boolean hoodOverride = false;
    private boolean flywheelOverride = false;
    private double hoodOffset = 0;
    private AngularVelocity flywheelOffset = RPM.zero();

    static SysIdRoutine routine;
    static {
        shootP.initDefault(ShooterConstants.SHOOTER_KP);
        shootD.initDefault(ShooterConstants.SHOOTER_KD);

        shootS.initDefault(ShooterConstants.SHOOTER_KS);
        shootV.initDefault(ShooterConstants.SHOOTER_KV);
        shootA.initDefault(ShooterConstants.SHOOTER_KA);
    }

    private Rotation2d robotAngle = Rotation2d.fromDegrees(0    );

    public Shooter(ShooterIO io) {
        this.io = io;
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setFlywheelVoltage, null, this));
    }

    @Override
    public void periodic() {

        if(shootP.hasChanged(hashCode()) || shootD.hasChanged(hashCode()) ||
            shootS.hasChanged(hashCode()) || shootV.hasChanged(hashCode()) || shootA.hasChanged(hashCode())) {
            io.setPID(shootP.get(), 0, shootD.get(), shootS.get(), shootV.get(), shootA.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        ShootingSolution solution = solver.getShootingSolution();
        hoodPercent = solution.hoodPercent/100;
        flywheelVelocity = RPM.of(solution.flywheelVelocity());
        robotAngle = solution.driveAngle();

        Logger.recordOutput("Shooter/CalculatedHoodPercent", hoodPercent);
        Logger.recordOutput("Shooter/CalculatedFlywheelVelocity", flywheelVelocity);
        Logger.recordOutput("Shooter/ActualFlywheelSpeed", inputs.flywheelVelocity.in(RPM));
        Logger.recordOutput("Shooter/CalculatedRobotAngle", robotAngle);
        Logger.recordOutput("Shooter/Distance", solution.distance());
        Logger.recordOutput("Shooter/DesiredState", desiredState.toString());
        Logger.recordOutput("Shooter/CurrentState", currentState.toString());
        if (currentState != desiredState) {
            switch (desiredState) {
                case CALIBRATING:
                    currentState=ShooterState.CALIBRATING;
                    break;
                case STATIONARY:
                    io.setFlywheelVelocity(RPM.of(0));
                    io.setHoodPercent(0)
                    ;
                    break;
                case IDLE:
                    io.setFlywheelVelocity(SHOOTER_IDLE);
                    io.setHoodPercent(0);
                    break;
                case SHOOTING:
                    io.setHoodPercent(hoodOverride ? hoodOffset : hoodPercent);
                    io.setFlywheelVelocity(flywheelVelocity);
                    break;
                case CENTERSHOOTING:
                    io.setFlywheelVelocity(RPM.of(3490));
                    io.setHoodPercent(7);
                    break;
                 case NonCameraShooting:
                    io.setHoodPercent(0);
                    io.setFlywheelVelocity(RPM.of(3175));// still need to set the value going to get it after testing;
                    break;
                default:
                    break;
                    // This case is used when the camera is broken / vision cannot estimate the hood distance, so we just set the hood to a fixed angle and the flywheel to a fixed velocity. Not ideal, but better used when the camera is broken;
                    // xbox to call this case
                
                    // case PassMode: 
                    // io.setHoodPercent(0);
                    // io.setFlywheelVelocity(RPM.of(0));//
                    // break;
            }
            
            if (inputs.atTargetAngle && inputs.atTargetVelocity) {
                currentState = desiredState;
            }

        }

        // This method will be called once per scheduler run
    }

    /**
     * Sets the desired state of the shooter.
     *
     * @param state The desired ShooterState.
     */
    public void setDesiredState(ShooterState state) {

        this.desiredState = state;

    }

    /** Returns the current state of the shooter.
     *
     * @return The current ShooterState.
     */
    public ShooterState getCurrentState() {
        return currentState;
    }

    /** Checks if the shooter is at the desired state.
     *
     * @return true if the current state matches the desired state, false otherwise.
     */

    public boolean isAtDesiredState() {
        return currentState == desiredState;
    }

    /** Possible states for the shooter subsystem. */
    public enum ShooterState {
        STATIONARY, IDLE, SHOOTING,CENTERSHOOTING,NonCameraShooting, CALIBRATING
    }

    public Rotation2d getWantedRobotAngle() {
        return robotAngle;
    }

    public void incrementFlywheelSpeed(double rpm) {
        flywheelOverride = true;
        flywheelOffset = flywheelOffset.plus(RPM.of(rpm));
    }

    public void incrementHoodAngle(double increment) {
        hoodOverride = true;
        hoodOffset = hoodOffset + increment;
    }

    public void resetOverrides() {
        hoodOverride = false;
        flywheelOverride = false;
        hoodOffset = 0;
        flywheelOffset = RPM.zero();
    }


    public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(()-> {
            desiredState = ShooterState.CALIBRATING;
            currentState = ShooterState.CALIBRATING;
        }).andThen(
            routine.quasistatic(direction));
    }

     public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
         return Commands.runOnce(()-> {
             desiredState = ShooterState.CALIBRATING;
             currentState = ShooterState.CALIBRATING;
         }).andThen(
             routine.dynamic(direction));
     }

    public void addTuningCommandsToAutoChooser(LoggedDashboardChooser<Command> chooser) {
        // These only apply to when we're doing "real" tuning
        if (Constants.tuningMode) {
            chooser.addOption("TUNING | Shooter SysId (Quasistatic Forward)",
                sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Shooter SysId (Quasistat.ic Reverse)",
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            chooser.addOption("TUNING | Shooter SysId (Dynamic Forward)",
                sysIdDynamic(SysIdRoutine.Direction.kForward));
            chooser.addOption("TUNING | Shooter SysId (Dynamic Reverse)",
                sysIdDynamic(SysIdRoutine.Direction.kReverse));
            chooser.addOption("TUNING | Shooter SysId ALL",
                Commands.runOnce(()-> {
                    desiredState = ShooterState.CALIBRATING;
                    currentState = ShooterState.CALIBRATING;
                }).andThen(Commands.sequence(
                    routine.quasistatic(SysIdRoutine.Direction.kForward),
                    Commands.waitSeconds(10),
                    routine.quasistatic(SysIdRoutine.Direction.kReverse),
                    Commands.waitSeconds(10),
                    routine.dynamic(SysIdRoutine.Direction.kForward),
                    Commands.waitSeconds(10),
                    routine.dynamic(SysIdRoutine.Direction.kReverse),
                    Commands.waitSeconds(10)
                )));
        }
    }
}
