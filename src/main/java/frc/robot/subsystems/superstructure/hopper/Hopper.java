// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem responsible for controlling the robot's hopper (ball/coin/wheel transporter).
 *
 * <p>This class implements a simple state machine to manage the hopper motor via a provided
 * HopperIO implementation. It exposes a small public surface:
 * - periodic(): executed once per scheduler run to progress transitions, apply motor outputs,
 *   and sample/log inputs.
 * - setDesiredState(HopperState): request a new hopper state; if different from the current
 *   state the subsystem enters the TRANSITIONING state and will apply the requested state on the
 *   next periodic() call.
 * - getCurrentState(): read the current state of the hopper state machine.
 *
 * <p>States and behavior:
 * - IDLE: motor is stopped.
 * - FEEDING: motor is driven forward using HopperConstants.HOPPER_VOLTAGE.
 * - REVERSING: motor is driven in reverse using the negated HopperConstants.HOPPER_VOLTAGE.
 * - TRANSITIONING: internal transient state used to atomically switch from one active state
 *   to another; callers should not set this state directly.
 *
 * <p>Responsibilities performed each scheduler run (periodic):
 * - If the subsystem is in TRANSITIONING, switch to the desired state and apply the
 *   corresponding motor command (stop / forward voltage / reverse voltage).
 * - Read hardware inputs via hopperIO.updateInputs(...) and log them through AdvantageKit
 *   (Logger.processInputs(...)).
 *
 * <p>Threading and usage notes:
 * - This subsystem is designed to be run from the WPILib scheduler thread (robot loop).
 * - setDesiredState(...) is intended to be called from commands or other robot code; if the
 *   requested state differs from the current state the subsystem will defer the actual motor
 *   change until the next periodic() invocation to keep state transitions deterministic.
 *
 * @see HopperIO
 * @see HopperConstants
 */
public class Hopper extends SubsystemBase {
	private final HopperIO hopperIO;
	private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
	private HopperState currentState = HopperState.IDLE;
	private HopperState desiredState = HopperState.IDLE;

	/** Creates a new Hopper. 
	 * @param hopperIO The IO implementation to use for the hopper
	*/
	public Hopper(HopperIO hopperIO) {
		this.hopperIO = hopperIO;
	}

	@Override
	public void periodic() {
		if (currentState == HopperState.TRANSITIONING) {
			switch (desiredState) {
			case IDLE:
				hopperIO.stopMotor();
				currentState = HopperState.IDLE;
				break;
			case FEEDING:
				hopperIO.setMotorVoltage(HopperConstants.HOPPER_VOLTAGE);
				currentState = HopperState.FEEDING;
				break;
			case REVERSING:
				hopperIO.setMotorVoltage(HopperConstants.HOPPER_VOLTAGE.unaryMinus());
				currentState = HopperState.REVERSING;
				break;
			default:
				hopperIO.stopMotor();
				currentState = HopperState.IDLE;
				break;
		}
		}
    // This method will be called once per scheduler run

	hopperIO.updateInputs(inputs); // Update inputs from hardware
	Logger.processInputs("Hopper", inputs); // Log inputs to AdvantageKit
	}
	/** Possible states for the hopper
	 * WARNING: Do not use transitioning state outside of Hopper subsystem
	 */
	public enum HopperState {
		IDLE,
		FEEDING,
		REVERSING,
		TRANSITIONING
	}
	public HopperState getCurrentState() {
		return currentState;
	}
	public void setDesiredState(HopperState state) {
		if (state != currentState) {
			this.desiredState = state;
			currentState = HopperState.TRANSITIONING;
		}
		
	}
}
