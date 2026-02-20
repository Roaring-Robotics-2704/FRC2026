// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_IDLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_TARGET;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.solvers.BasicTunedCalc;
import frc.robot.util.solvers.SolverIO;

/** The shooter subsystem. */
public class Shooter extends SubsystemBase {

    private ShooterState currentState = ShooterState.STATIONARY;
    private ShooterState desiredState = ShooterState.IDLE;

    private Angle hoodAngle;
    private AngularVelocity flywheelVelocity;


    private final ShooterIO io;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SolverIO solver;

    public Shooter(ShooterIO io, SolverIO solver) {
        this.io = io;
        this.solver = solver;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        hoodAngle = solver.getShootingSolution(RobotState.getInstance().getOdometryPose(), RobotState.getInstance().getRobotVelocity()).hoodAngle();
        flywheelVelocity = solver.getShootingSolution(RobotState.getInstance().getOdometryPose(), RobotState.getInstance().getRobotVelocity()).flywheelVelocity();
        Logger.recordOutput("Shooter/CalculatedHoodAngle", hoodAngle);
        Logger.recordOutput("Shooter/CalculatedFlywheelVelocity", flywheelVelocity);

        Logger.recordOutput("Shooter/DesiredState", desiredState.toString());
        Logger.recordOutput("Shooter/CurrentState", currentState.toString());

        if (currentState != desiredState) {
            switch (desiredState) {
                case STATIONARY:
                    io.setFlywheelVoltage(Volts.of(0));
                    io.setHoodAngle(MIN_ANGLE);
                    ;
                    break;
                case IDLE:
                    io.setFlywheelVelocity(SHOOTER_IDLE);
                    io.setHoodAngle(MIN_ANGLE);
                    break;
                case SHOOTING:
                    io.setHoodAngle(hoodAngle);
                    io.setFlywheelVelocity(SHOOTER_TARGET);
                    break;
                default:
                    break;
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
        STATIONARY, IDLE, SHOOTING
    }

}
