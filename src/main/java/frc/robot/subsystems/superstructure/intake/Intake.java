// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MAX_DISTANCE;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Intake subsystem for controlling the robot's intake mechanism. */
public class Intake extends SubsystemBase {
    private IntakeState currentState = IntakeState.INSIDE;
    private IntakeState desiredState = IntakeState.INSIDE;
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private Timer calibrationTimer = new Timer();
    private Distance foundMaxDistance = Inches.zero();

    /** Creates a new Intake. */
    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        // This method will be called once per scheduler run
        if (currentState != desiredState) {
            switch (desiredState) {
                case INSIDE:
                    intakeIO.setPosition(Inches.zero());
                    intakeIO.setRollerVoltage(Volts.of(0));
                    break;
                case STOWED:
                    intakeIO.setPosition(Inches.of(4)); // Example stowed position
                    intakeIO.setRollerVoltage(Volts.of(0));
                    break;
                case DEPLOYED_OFF:
                    intakeIO.setPosition(SLIDE_MAX_DISTANCE); // Example deployed position
                    intakeIO.setRollerVoltage(Volts.of(0));
                    break;
                case DEPLOYED_ON:
                    intakeIO.setPosition(SLIDE_MAX_DISTANCE); // Example deployed position
                    intakeIO.setRollerVoltage(Volts.of(6)); // Example roller voltage to
                    break;
                case CALIBRATE_OUT:
                    intakeIO.setSlideVoltage(Volts.of(4));
                    intakeIO.setRollerVoltage(Volts.of(0));
                    calibrationTimer.start();
                    break;
                case CALIBRATE_IN:
                    intakeIO.setSlideVoltage(Volts.of(-4));
                    intakeIO.setRollerVoltage(Volts.of(0));
                    calibrationTimer.start();
                    break;
                default:
                    break;
            }
            intakeIO.updateInputs(inputs);
            if (desiredState.isCalibrating()) {
                if (Math.abs(inputs.slideVelocity.in(InchesPerSecond)) > 0.001) {
                    calibrationTimer.reset();
                    calibrationTimer.start();
                } else if (calibrationTimer.hasElapsed(3)) {
                    if (desiredState == IntakeState.CALIBRATE_IN) {
                        intakeIO.resetSlideEncoder(Inches.zero());
                    } else if (desiredState == IntakeState.CALIBRATE_OUT) {
                        foundMaxDistance = Inches.of(inputs.slidePosition.in(Inches));
                        System.out.println("Found max distance: " + foundMaxDistance);
                    }
                    currentState = IntakeState.INSIDE;
                    calibrationTimer.stop();
                    calibrationTimer.reset();
                }
            } else if (inputs.slideAtPosition) {
                currentState = desiredState;
            }
        }

    }

    public void setDesiredState(IntakeState state) {
        this.desiredState = state;
    }

    /** Possible goals for the intake subsystem. */
    public enum IntakeState {
        INSIDE(false),
        STOWED(false),
        DEPLOYED_OFF(false),
        DEPLOYED_ON(false),
        CALIBRATE_OUT(true),
        CALIBRATE_IN(true);

        boolean calibrating;
        
        /** Constructor for IntakeState enum. */
        private IntakeState(boolean calibrating) {
            this.calibrating = calibrating;
        }

        public boolean isCalibrating() {
            return calibrating;
        }
    }

    public boolean atDesiredState() {
        return currentState == desiredState;
    }
}
