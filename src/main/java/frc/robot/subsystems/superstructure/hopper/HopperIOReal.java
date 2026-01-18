// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_CURRENT_LIMIT;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkUtil;

/** Real implementation of the hopper */
public class HopperIOReal implements HopperIO {
	SparkMax hopperMotor = new SparkMax(10, SparkMax.MotorType.kBrushless);

	public HopperIOReal() {
		SparkMaxConfig hopperConfig = new SparkMaxConfig();
		hopperConfig.idleMode(IdleMode.kCoast);
		hopperConfig.smartCurrentLimit(HOPPER_CURRENT_LIMIT);
		SparkUtil.tryUntilOk(hopperMotor,5, () -> hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

	}

	@Override
	public void updateInputs(HopperIOInputs inputs) { 
		inputs.currentDraw.mut_replace(Amps.of(hopperMotor.getOutputCurrent()));
		inputs.appliedVoltage.mut_replace(Volts.of(hopperMotor.getAppliedOutput()));
		inputs.motorVelocity.mut_replace(RotationsPerSecond.of(hopperMotor.getEncoder().getVelocity()));
	}
	@Override
	public void setMotorVoltage(Voltage voltage) {
		hopperMotor.setVoltage(voltage);
	}
	@Override
	public void stopMotor() {
		hopperMotor.stopMotor();
	}
	
}