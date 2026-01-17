// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
