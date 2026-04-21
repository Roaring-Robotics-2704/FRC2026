// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {
    SparkMax climberMotor;
    SparkMax hookMotor;

    public ClimberIOReal() {
        climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, SparkMax.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(
                ClimberConstants.CLIMBER_STALL_CURRENT_LIMIT, ClimberConstants.CLIMBER_FREE_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(
                ClimberConstants.CLIMBER_GEARING
                        * ClimberConstants.CLIMBER_PULLEY_CIRCUMFERENCE.in(Inches));
        config.inverted(true);
        SparkUtil.tryUntilOk(
                climberMotor,
                5,
                () -> climberMotor.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        hookMotor = new SparkMax(ClimberConstants.HOOK_MOTOR_ID, SparkMax.MotorType.kBrushless);
        SparkMaxConfig hookConfig = new SparkMaxConfig();
        hookConfig.smartCurrentLimit(
                ClimberConstants.HOOK_STALL_CURRENT_LIMIT, ClimberConstants.HOOK_FREE_CURRENT_LIMIT);
        hookConfig.idleMode(IdleMode.kCoast);

        SparkUtil.tryUntilOk(
                hookMotor,
                5,
                () -> hookMotor.configure(
                        hookConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberMotor.setVoltage(voltage);
    }

    @Override
    public void resetClimberEncoder() {
        climberMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setHookVoltage(double voltage) {
        hookMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPosition.mut_replace(climberMotor.getEncoder().getPosition(), Inches);
        inputs.climberCurrent.mut_replace(climberMotor.getOutputCurrent(), Amps);
        inputs.hookCurrent.mut_replace(hookMotor.getOutputCurrent(), Amps);
        inputs.climberVoltage.mut_replace(hookMotor.getBusVoltage(), Volts);
        inputs.climberVelocity.mut_replace(climberMotor.getEncoder().getVelocity(), InchesPerSecond);
    }
}
