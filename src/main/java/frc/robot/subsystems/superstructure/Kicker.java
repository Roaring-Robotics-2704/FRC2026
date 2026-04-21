// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil;

public class Kicker extends SubsystemBase {
    Voltage desiredVoltage = Volts.of(-1);
    Timer timer = new Timer();
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
        // if (Math.abs(kickerMotor.getEncoder().getVelocity()) < 0.0005 && desiredVoltage != Volts.of(0)) {
        //     if (!timer.isRunning()) {
        //         timer.reset();
        //         timer.start();
        //     }
        //     kickerMotor.setVoltage(-1);
        // } else if (timer.isRunning() && timer.get() < 1) {
        //     kickerMotor.setVoltage(-1);
        // } else {
        //     timer.stop();
        kickerMotor.setVoltage(desiredVoltage);
        // }
        Logger.recordOutput("Kicker/Current", kickerMotor.getOutputCurrent());
        Logger.recordOutput("Kicker/DesiredVoltage", kickerMotor.getAppliedOutput());
        Logger.recordOutput("Kicker/Velocity", kickerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Kicker/Timer", timer.get());
        Logger.recordOutput("Kicker/IsTimerRunning", timer.isRunning());
        
    }

    public void setKickerVoltage(double voltage) {
        desiredVoltage = Volts.of(voltage);
    }

    public void stopKicker() {
        desiredVoltage = Volts.of(0);
    }

}
