// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  Climbing States Calibrating -> full power to bring the climber down - once velocity is zero we
  know it's at the bottom (alt: once current spikes) - units (distance mutable inches) Stopped -> No
  power in Motor Transition to Top -> 6 Volts going CCW until height reached or timeout of 5
  seconds Transition to Middle -> Find the current height. If less than middle. 6V CCW until height
  in tolerance range. Else opp. Transition to Botton -> 6 Volts going CW until height reached or
  timeout of 5 seconds
 */
package frc.robot.subsystems.superstructure.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static frc.robot.subsystems.superstructure.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO climberIO;
    private final Timer timeoutTimer = new Timer();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    boolean hasCalibrated = false;

    private ClimberState currentState = ClimberState.BOTTOM;
    private ClimberState desiredState = ClimberState.BOTTOM;

    /** Creates a new Climber. */
    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        if (!hasCalibrated && DriverStation.isEnabled()) {
            desiredState = ClimberState.CALIBRATING;
        }
        if (DriverStation.isDisabled()) {
            timeoutTimer.reset();
        }
        if (currentState != desiredState) {
            switch (desiredState) {
                case CALIBRATING:
                    startTimer();
                    climberIO.setClimberVoltage(-1);
                    climberIO.setHookVoltage(0);
                    if (((inputs.climberVelocity.in(InchesPerSecond) == 0.0)
                            || inputs.climberCurrent.in(Amps) > 15)
                            && timeoutTimer.hasElapsed(0.5) && DriverStation.isEnabled()) {
                        climberIO.setClimberVoltage(0);
                        climberIO.resetClimberEncoder();
                        stopTimer();
                        hasCalibrated = true;
                        desiredState = ClimberState.BOTTOM;
                        currentState = ClimberState.BOTTOM;
                    }

                    break;
                case BOTTOM:
                    startTimer();
                    if (inputs.climberPosition.in(Inches) > MID_CLIMBER_HEIGHT.in(Inches)) {
                        climberIO.setClimberVoltage(-2);
                    } else {
                        climberIO.setClimberVoltage(2);
                    }
                    climberIO.setHookVoltage(0);
                    if (MathUtil.isNear(
                            inputs.climberPosition.in(Inches),
                            MIN_CLIMBER_HEIGHT.in(Inches),
                            CLIMBER_TOLERANCE.in(Inches))
                            || timeoutTimer.hasElapsed(TIMEOUT)) {
                        climberIO.setClimberVoltage(0);
                        stopTimer();
                        currentState = ClimberState.BOTTOM;
                    }
                    break;
                case MIDDLE:
                    startTimer();
                    climberIO.setHookVoltage(1);
                    if (inputs.climberPosition.in(Inches) > MID_CLIMBER_HEIGHT.in(Inches)) {
                        climberIO.setClimberVoltage(-2);

                    } else {
                        climberIO.setClimberVoltage(2);
                    }
                    if (MathUtil.isNear(
                            inputs.climberPosition.in(Inches),
                            MID_CLIMBER_HEIGHT.in(Inches),
                            CLIMBER_TOLERANCE.in(Inches))
                            || timeoutTimer.hasElapsed(TIMEOUT)) {
                        climberIO.setClimberVoltage(0);
                        stopTimer();
                        currentState = ClimberState.MIDDLE;
                    }
                    break;
                case TOP:
                    startTimer();
                    if (inputs.climberPosition.in(Inches) > MID_CLIMBER_HEIGHT.in(Inches)) {
                        climberIO.setClimberVoltage(-2);

                    } else {
                        climberIO.setClimberVoltage(2);
                    }
                    climberIO.setHookVoltage(1);
                    if (MathUtil.isNear(
                            inputs.climberPosition.in(Inches),
                            MAX_CLIMBER_HEIGHT.in(Inches),
                            CLIMBER_TOLERANCE.in(Inches))
                            || timeoutTimer.hasElapsed(TIMEOUT)) {
                        climberIO.setClimberVoltage(0);
                        stopTimer();
                        currentState = ClimberState.TOP;
                    }
                    break;
                default:
                    break;
            }
        }
        Logger.recordOutput("Climber/DesiredState", desiredState);
        Logger.recordOutput("Climber/CurrentState", currentState);
        Logger.recordOutput("Climber/hasCalibrated", hasCalibrated);
        // This method will be called once per scheduler run
    }

    public enum ClimberState {
        CALIBRATING, BOTTOM, MIDDLE, TOP,
    }

    private void startTimer() {
        if (!timeoutTimer.isRunning()) {
            timeoutTimer.reset();
            timeoutTimer.start();
        }
    }

    private void stopTimer() {
        timeoutTimer.stop();
        timeoutTimer.reset();
    }

    public void setDesiredState(ClimberState desiredState) {
        this.desiredState = desiredState;
    }

    public boolean isAtDesiredState() {
        return currentState == desiredState;
    }

    public Command goToStateCommand(ClimberState state) {
        return Commands.run(() -> setDesiredState(state), this).until(this::isAtDesiredState);
    }
}
