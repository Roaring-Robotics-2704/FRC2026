// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Kicker extends SubsystemBase {
    Voltage desiredVoltage = Volts.of(0);
    /** Creates a new Kicker. */
    SparkMax kickerMotor = new SparkMax(31, SparkMax.MotorType.kBrushless);

    public Kicker() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        SparkUtil.tryUntilOk(kickerMotor, 5,
                () -> kickerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void periodic() {
        if (kickerMotor.getOutputCurrent() > 20) {
            kickerMotor.setVoltage(-1);
        } else {
            kickerMotor.setVoltage(desiredVoltage);
        }
    }

    public void setKickerVoltage(double voltage) {
        desiredVoltage = Volts.of(voltage);
    }

    public void stopKicker() {
        desiredVoltage = Volts.of(0);
    }

}
