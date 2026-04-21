// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hook;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.hook.HookConstants.HOOK_MOTOR_ID;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;


/** Real implementation of the hopper. */
public class HookIOReal implements HookIO {

    SparkMax hookMotor = new SparkMax(HOOK_MOTOR_ID, SparkMax.MotorType.kBrushless);

    /** Instantiates the Real Hopper hardware. */
    public HookIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
    }

    @Override
    public void updateInputs(HookIOInputs inputs) {
        inputs.hookCurrent.mut_replace(Amps.of(hookMotor.getOutputCurrent()));
        inputs.hookVoltage.mut_replace(Volts.of(hookMotor.getAppliedOutput()));
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        hookMotor.setVoltage(voltage);
    }
}
    