// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tunables.LoggedTunableNumber;

/** Intake subsystem for controlling the robot's intake mechanism. */
public class Intake extends SubsystemBase {
    // private IntakeState currentState = IntakeState.INSIDE;
    // private IntakeState desiredState = IntakeState.INSIDE;
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    // private Timer calibrationTimer = new Timer();
    // private Distance foundMaxDistance = Inches.zero();
    // LinearFilter currentFilter = LinearFilter.movingAverage( (int)(1/0.02));
    // private double current;
    // private boolean isAtDesiredState = true;

    /*
     * private static final LoggedTunableNumber slideP = new
     * LoggedTunableNumber("Intake/SlideP");
     * private static final LoggedTunableNumber slideD = new
     * LoggedTunableNumber("Intake/SlideD");
     * 
     * private static final LoggedTunableNumber slideS = new
     * LoggedTunableNumber("Intake/SlideS");
     * private static final LoggedTunableNumber slideV = new
     * LoggedTunableNumber("Intake/SlideV");
     * private static final LoggedTunableNumber slideA = new
     * LoggedTunableNumber("Intake/SlideA");
     */

    /*
     * static {
     * slideP.initDefault(IntakeConstants.SLIDE_POSITION_KP);
     * slideD.initDefault(IntakeConstants.SLIDE_POSITION_KD);
     * 
     * slideS.initDefault(IntakeConstants.SLIDE_POSITION_KS);
     * slideV.initDefault(IntakeConstants.SLIDE_POSITION_KV);
     * slideA.initDefault(IntakeConstants.SLIDE_POSITION_KA);
     * }
     */
    /** Creates a new Intake. */
    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;

    }

    public void setRollersEnabled(boolean enabled) {
        intakeIO.setRollerVoltage(Volts.of(enabled ? 10 : 0));
    }

    private void setSlideVoltage(double volts) {
        intakeIO.setSlideVoltage(Volts.of(volts));
    }

    public Command extendSlide() {
        return Commands.run(() -> setSlideVoltage(6));
    }

    public Command retractSlide() {
        return Commands.run(() -> setSlideVoltage(-6));
    }
    public Command stopIntake() {
        return Commands.run(() -> setSlideVoltage(0));
    }
    
    public Command intake() {
        return Commands.parallel(
            extendSlide().withTimeout(2),
            Commands.startEnd(() -> setRollersEnabled(true), () -> setRollersEnabled(false))
        ).finallyDo(this::stopIntake);
    }

    public Command retract() {
        return Commands.parallel(
            retractSlide(),
            Commands.startEnd(() -> setRollersEnabled(true), () -> setRollersEnabled(false))
        ).finallyDo(this::stopIntake);
    }


    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {
        /*
         * if(slideP.hasChanged(hashCode()) || slideD.hasChanged(hashCode()) ||
         * slideS.hasChanged(hashCode()) || slideV.hasChanged(hashCode()) ||
         * slideA.hasChanged(hashCode())) {
         * intakeIO.setPID(slideP.get(), slideD.get(), slideS.get(), slideV.get(),
         * slideA.get());
         * }
         */
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        // This method will be called once per scheduler run
        // current = currentFilter.calculate(Math.abs(inputs.slideCurrentDraw.in(Amp)));
        // if (desiredState != currentState) {
        //     switch (desiredState) {
        //         case INSIDE:
        //             intakeIO.setSlideVoltage(Volts.of(4));
        //             intakeIO.setRollerVoltage(Volts.of(0));
        //             break;
        //         case DEPLOYED_OFF:
        //             intakeIO.setSlideVoltage(Volts.of(-6));
        //             intakeIO.setRollerVoltage(Volts.of(0));
        //             break;
        //         case DEPLOYED_ON:
        //             intakeIO.setSlideVoltage(Volts.of(-6));
        //             intakeIO.setRollerVoltage(Volts.of(12)); // Example roller voltage to
        //             break;
        //         default:
        //             break;
        //     }
        //     if ((current) >= IntakeConstants.SLIDE_STALL_LIMIT) {
        //         currentState = desiredState;
        //         intakeIO.stopMotors();
        //         isAtDesiredState = true;
        //     }
        // }

        // Logger.recordOutput("Intake/CurrentState", currentState);
        // Logger.recordOutput("Intake/DesiredState", desiredState);
        // Logger.recordOutput("Intake/Current", current);
        // Logger.recordOutput("Intake/IsAtDesiredState", isAtDesiredState);
    }

    // public void setDesiredState(IntakeState state) {
    //     this.desiredState = state;
    // }

    // /** Possible goals for the intake subsystem. */
    // public enum IntakeState {
    //     INSIDE, DEPLOYED_OFF, DEPLOYED_ON;
    // }

    // public enum IntakePosition {
    //     EXTENDED, RETRACTED, OFF;
    // }

    // public boolean atDesiredState() {
    //     // return currentState == desiredState;
    //     return isAtDesiredState;
    // }

}
