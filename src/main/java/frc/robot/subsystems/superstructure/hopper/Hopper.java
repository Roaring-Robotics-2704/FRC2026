// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_VOLTAGE;
import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_VOLTAGE_RAMP_RATE;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem responsible for controlling the robot's hopper (ball/coin/wheel
 * transporter).
 *
 * <p>This class implements a simple state machine to manage the hopper motor via a
 * provided
 * HopperIO implementation. It exposes a small public surface:
 * - periodic(): executed once per scheduler run to progress transitions, apply
 * motor outputs,
 * and sample/log inputs.
 * - setDesiredState(HopperState): request a new hopper state; if different from
 * the current
 * state the subsystem enters the TRANSITIONING state and will apply the
 * requested state on the
 * next periodic() call.
 * - getCurrentState(): read the current state of the hopper state machine.
 *
 * <p>States and behavior:
 * - IDLE: motor is stopped.
 * - FEEDING: motor is driven forward using HopperConstants.HOPPER_VOLTAGE.
 * - REVERSING: motor is driven in reverse using the negated
 * HopperConstants.HOPPER_VOLTAGE.
 * - TRANSITIONING: internal transient state used to atomically switch from one
 * active state
 * to another; callers should not set this state directly.
 *
 * <p>Responsibilities performed each scheduler run (periodic):
 * - If the subsystem is in TRANSITIONING, switch to the desired state and apply
 * the
 * corresponding motor command (stop / forward voltage / reverse voltage).
 * - Read hardware inputs via hopperIO.updateInputs(...) and log them through
 * AdvantageKit
 * (Logger.processInputs(...)).
 *
 * <p>Threading and usage notes:
 * - This subsystem is designed to be run from the WPILib scheduler thread
 * (robot loop).
 * - setDesiredState(...) is intended to be called from commands or other robot
 * code; if the
 * requested state differs from the current state the subsystem will defer the
 * actual motor
 * change until the next periodic() invocation to keep state transitions
 * deterministic.
 *
 * @see HopperIO
 * @see HopperConstants
 */
public class Hopper extends SubsystemBase {
    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
    private HopperState currentState = HopperState.IDLE;
    private HopperState desiredState = HopperState.IDLE;
    private SlewRateLimiter voltageSlewRateLimiter = new SlewRateLimiter(HOPPER_VOLTAGE_RAMP_RATE);

    Double targetVolts;
    Double rampedVolts;


    /**
     * Creates a new Hopper.
     *
     * @param hopperIO The IO implementation to use for the hopper
     *
     */
    public Hopper(HopperIO hopperIO) {
        this.hopperIO = hopperIO;
    }

    @Override
    public void periodic() {
        if (currentState == HopperState.TRANSITIONING) {
            // Determine numeric target volts based on desired state
            switch (desiredState) {
                case IDLE:
                    targetVolts = 0.0;
                    break;
                case FEEDING:
                    targetVolts = HOPPER_VOLTAGE.in(Volts);
                    break;
                case REVERSING:
                    targetVolts = -HOPPER_VOLTAGE.in(Volts);
                    break;
                default:
                    targetVolts = 0.0;
                    break;
            }

            // Ramp toward the target each scheduler run and command the ramped value
            rampedVolts = voltageSlewRateLimiter.calculate(targetVolts);
            hopperIO.setMotorVoltage(Volts.of(rampedVolts));

            // Only leave TRANSITIONING once we're effectively at the setpoint
            final double EPS_V = 0.01; // volts tolerance
            if (Math.abs(rampedVolts - targetVolts) <= EPS_V) {
                // Ensure final commanded value is the exact setpoint (or stopped)
                switch (desiredState) {
                    case IDLE:
                        hopperIO.stopMotor();
                        break;
                    case FEEDING:
                        hopperIO.setMotorVoltage(HOPPER_VOLTAGE);
                        break;
                    case REVERSING:
                        hopperIO.setMotorVoltage(Volts.of(-HOPPER_VOLTAGE.in(Volts)));
                        break;
                    default:
                        hopperIO.stopMotor();
                        break;
                }

                currentState = desiredState;
            }
        }
        // This method will be called once per scheduler run

        hopperIO.updateInputs(inputs); // Update inputs from hardware
        Logger.processInputs("Hopper", inputs); // Log inputs to AdvantageKit
    }

    /**
     * Possible states for the hopper.
     * WARNING: Do not use transitioning state outside of Hopper subsystem
     */
    public enum HopperState {
        IDLE,
        FEEDING,
        REVERSING,
        TRANSITIONING
    }

    /**
     * Gets the current state of the hopper.
     *
     * @return The current HopperState
     */
    public HopperState getCurrentState() {
        return currentState;
    }

    /**
     * Sets the desired state of the hopper. If the desired state differs from the
     * current state,
     * the hopper will enter the TRANSITIONING state and apply the new state on the
     * next periodic() call.
     *
     * @param state The desired HopperState
     */
    public void setDesiredState(HopperState state) {
        if (state != currentState) {
            this.desiredState = state;
            currentState = HopperState.TRANSITIONING;
        }

    }
}
