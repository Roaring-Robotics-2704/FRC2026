// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_CURRENT_LIMIT;
import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_MOTOR_ID;
import static frc.robot.subsystems.superstructure.hopper.HopperConstants.HOPPER_MOTOR_TYPE;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkUtil;

/** Real implementation of the hopper. */
public class HopperIOSim implements HopperIO {

    Voltage targetVoltage = Volts.of(0);
    // SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, SparkMax.MotorType.kBrushless);
    // SparkMaxSim hopperMotorSim;

    /** Instantiates the Real Hopper hardware. */
    public HopperIOSim() {
        // SparkMaxConfig hopperConfig = new SparkMaxConfig();
        // hopperConfig.idleMode(IdleMode.kCoast);
        // hopperConfig.smartCurrentLimit(HOPPER_CURRENT_LIMIT);
        // SparkUtil.tryUntilOk(hopperMotor, 5,
        //     () -> hopperMotor.configure(hopperConfig,
        //         ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters)
        // );
        // hopperMotorSim = new SparkMaxSim(hopperMotor, HOPPER_MOTOR_TYPE);
        // hopperMotorSim.enable();
        
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        // hopperMotorSim.iterate(hopperMotorSim.getVelocity(), 12, 0.02);
        // inputs.currentDraw.mut_replace(Amps.of(hopperMotorSim.getMotorCurrent()));
        // inputs.appliedVoltage.mut_replace(Volts.of(hopperMotorSim.getAppliedOutput()));
        // inputs.motorVelocity.mut_replace(RotationsPerSecond.of(
        //     hopperMotorSim.getVelocity()
        // ));
        inputs.appliedVoltage.mut_replace(targetVoltage);
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        // hopperMotorSim.setAppliedOutput(MathUtil.clamp(voltage.in(Volts) / 12.0, -1.0, 1.0));
        targetVoltage = voltage;
    }

    @Override
    public void stopMotor() {
        targetVoltage = Volts.of(0);
        // hopperMotorSim.setAppliedOutput(0.0);
    }

}