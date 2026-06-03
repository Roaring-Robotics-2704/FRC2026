// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//port frc.robot.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
    Voltage desiredVoltage = Volts.of(-2);
    Timer timer = new Timer();
    /** Creates a new Kicker. */
    TalonFX kickerMotor = new TalonFX(31);

    public Kicker() {
       MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(true);
         TalonFXConfiguration config = new TalonFXConfiguration()
                 .withMotorOutput(
                        motorOutput)
                .withCurrentLimits(
                        currentLimits);
        //setKickerVoltage(-2);
        try {kickerMotor.getConfigurator().apply(config);}
        catch (Exception exception) {}
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
        //kickerMotor.setVoltage(-2);
        // // }
        // Logger.recordOutput("Kicker/Current", kickerMotor.getOutputCurrent());
        // Logger.recordOutput("Kicker/DesiredVoltage", kickerMotor.getMotorVoltage());
        // Logger.recordOutput("Kicker/Velocity", kickerMotor.getVelocity());
        // Logger.recordOutput("Kicker/Timer", timer.get());
        // Logger.recordOutput("Kicker/IsTimerRunning", timer.isRunning());

    }



    public void setKickerVoltage(double voltage) {
        desiredVoltage = Volts.of(voltage);
        kickerMotor.setVoltage(voltage);
    }

    public void stopKicker() {
        desiredVoltage = Volts.of(0);
        kickerMotor.setVoltage(0);
    }

}
