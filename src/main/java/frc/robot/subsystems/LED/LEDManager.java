// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDManager extends SubsystemBase {
    private LEDState currentState = LEDState.IDLE;
    private LEDState previousState = null;

    public enum LEDState {
        IDLE, INTAKE, SHOOTING, CLIMBING
    }

    AddressableLED ledStrip;
    AddressableLEDBuffer ledBuffer;

    LEDPattern shootPattern;
    LEDPattern intakePattern;
    LEDPattern climbPattern;
    LEDPattern idlePattern;

    public LEDManager() {
        ledStrip = new AddressableLED(8);
        ledBuffer = new AddressableLEDBuffer(92); // Change 60 to the actual number of LEDs
        ledStrip.setLength(ledBuffer.getLength());
        setupPatterns();
        idlePattern.applyTo(ledBuffer);

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case SHOOTING:
                shootPattern.applyTo(ledBuffer);
                break;
            case INTAKE:
                intakePattern.applyTo(ledBuffer);
                break;
            case CLIMBING:
                climbPattern.applyTo(ledBuffer);
                break;
            case IDLE:
                idlePattern.applyTo(ledBuffer);
                break;
            default:
                idlePattern.applyTo(ledBuffer);
                break;
        }
        previousState = currentState;

        ledStrip.setData(ledBuffer);
        // This method will be called once per scheduler run
    }

    private void setupPatterns() {
        shootPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kGreen);
        shootPattern = shootPattern.blink(Seconds.of(0.25));
        intakePattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kMagenta);
        intakePattern = intakePattern.blink(Seconds.of(0.25));
        climbPattern = LEDPattern.solid(Color.kAquamarine);
        idlePattern = LEDPattern.steps(Map.of(0,Color.kYellow,0.5,Color.kBlue))
                .scrollAtRelativeSpeed(Percent.per(Seconds).of(25));
    }

    public void setPattern(LEDState state) {
        this.currentState = state;
    }

}
