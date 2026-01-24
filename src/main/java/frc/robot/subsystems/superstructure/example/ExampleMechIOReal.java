// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.example;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotContainer;
import frc.robot.util.CANBusManager;
import frc.robot.util.PhoenixUtil;

import static frc.robot.subsystems.superstructure.example.ExampleMechConstants.MOTOR_ID;

/** Add your docs here. */
public class ExampleMechIOReal implements ExampleMechIO {
    private TalonFX motor;

    public ExampleMechIOReal() {
        motor = new TalonFX(MOTOR_ID, CANBusManager.getInstance().getRIOBus());
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Configure motor settings here as needed
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    }

    @Override
    public void updateInputs(ExampleMechIOInputs inputs) {


}
