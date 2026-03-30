// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.FLYWHEEL_MOTOR_ONE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.FLYWHEEL_MOTOR_TWO;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.HOOD_SERVO1_PORT;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MAX_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MIN_ANGLE;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KA;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KD;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KP;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KS;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KV;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_TOLERANCE;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ShooterIOGreyT implements ShooterIO {
    private StatusSignal<AngularVelocity> flywheelVelocitySignal;
    private StatusSignal<AngularAcceleration> flywheelAccelerationSignal;
    private StatusSignal<Voltage> leftFlywheelAppliedVoltsSignal;
    private StatusSignal<Voltage> rightFlyWheelAppliedVoltsSignal;
    private StatusSignal<Current> leftFlywheelCurrentAmpsSignal;
    private StatusSignal<Current> rightFlywheelCurrentAmpsSignal;
    private StatusSignal<Temperature> rightTempSignal;
    private StatusSignal<Temperature> leftTempSignal;

    private StatusSignalCollection statusSignals = new StatusSignalCollection();

    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    private double targetFlywheelVelocity = 0.0;

    /** Instantiates the GreyT Shooter hardware. */
    private LinearServo hoodServo1 = new LinearServo(HOOD_SERVO1_PORT,50, 6);

    /** Constructs the GreyT shooter code. */
    public ShooterIOGreyT() {
        flywheelMotor1 = new TalonFX(FLYWHEEL_MOTOR_ONE);
        flywheelMotor2 = new TalonFX(FLYWHEEL_MOTOR_TWO);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(true);

        Slot0Configs pidConfigs = new Slot0Configs()
                .withKP(SHOOTER_KP)
                .withKD(SHOOTER_KD)
                .withKV(SHOOTER_KV)
                .withKA(SHOOTER_KA)
                .withKS(SHOOTER_KS);
        // in init function
// var talonFXConfigs = new TalonFXConfiguration();
// // set slot 0 gains
// var slot0Configs = talonFXConfigs.Slot0;
// slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
// slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
// slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
// slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
// slot0Configs.kI = 0; // no output for integrated error
// slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
// //TODO set magic motion values 
// //set Motion Magic settings
// MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
// motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
// motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
// motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

// flywheelMotor1.getConfigurator().apply(talonFXConfigs);
// flywheelMotor2.getConfigurator().apply(talonFXConfigs);

        AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true).withBeepOnConfig(true).withBeepOnBoot(true);

        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        motorOutput)
                .withCurrentLimits(
                        currentLimits)
                .withSlot0(
                        pidConfigs).withAudio(audioConfigs);

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor1.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor2.getConfigurator().apply(config));

        flywheelVelocitySignal = flywheelMotor1.getVelocity();
        flywheelAccelerationSignal = flywheelMotor1.getAcceleration();
        leftFlywheelAppliedVoltsSignal = flywheelMotor1.getMotorVoltage();
        rightFlyWheelAppliedVoltsSignal = flywheelMotor2.getMotorVoltage();
        leftFlywheelCurrentAmpsSignal = flywheelMotor1.getStatorCurrent();
        rightFlywheelCurrentAmpsSignal = flywheelMotor2.getStatorCurrent();
        leftTempSignal = flywheelMotor1.getDeviceTemp();
        rightTempSignal = flywheelMotor2.getDeviceTemp();
       statusSignals.addSignals(
                flywheelVelocitySignal,
                flywheelAccelerationSignal,
                leftFlywheelAppliedVoltsSignal,
                rightFlyWheelAppliedVoltsSignal,
                leftFlywheelAppliedVoltsSignal,
                rightFlywheelCurrentAmpsSignal,
                leftTempSignal,
                rightTempSignal);

        statusSignals.setUpdateFrequencyForAll(Hertz.of(50));
        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor1, flywheelMotor2);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        statusSignals.refreshAll();
        inputs.flywheelVelocity.mut_replace(flywheelVelocitySignal.getValue());
        inputs.flywheelAcceleration.mut_replace(flywheelAccelerationSignal.getValue());
        inputs.leftFlywheelAppliedVolts.mut_replace(leftFlywheelAppliedVoltsSignal.getValue());
        inputs.rightFlywheelAppliedVolts.mut_replace(rightFlyWheelAppliedVoltsSignal.getValue());
        inputs.leftFlywheelCurrentAmps.mut_replace(leftFlywheelCurrentAmpsSignal.getValue());
        inputs.rightFlywheelCurrentAmps.mut_replace(rightFlywheelCurrentAmpsSignal.getValue());

        inputs.lefTemp.mut_replace(leftTempSignal.getValue());
        inputs.rightTemp.mut_replace(rightTempSignal.getValue());

        inputs.hoodAngle.mut_replace(MAX_ANGLE.minus(MIN_ANGLE).times(hoodServo1.get()));
        inputs.atTargetVelocity = flywheelMotor1.getClosedLoopError().getValue() < SHOOTER_TOLERANCE.in(RotationsPerSecond);
        inputs.atTargetAngle = hoodServo1.isFinished();
        inputs.targetFlywheelVelocity.mut_replace(targetFlywheelVelocity , RotationsPerSecond);
    }

    /**
     * Commands the shooter flywheel to spin at the specified angular velocity.
     *
     * @param velocity
     *            The target angular velocity for the flywheel.
     */
    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelMotor1.setControl(new VelocityVoltage(velocity));
        flywheelMotor2.setControl(new VelocityVoltage(velocity.unaryMinus()));

        targetFlywheelVelocity = velocity.in(RotationsPerSecond);

    }

    /**
     * Sets the hood angle by converting the desired angle to servo positions.
     *
     * @param angle
     *            The target angle for the hood.
     */
    @Override
    public void setHoodPercent(double percent) {
        double servoPosition = percent;
        servoPosition = MathUtil.clamp(servoPosition, 0, 1);
        hoodServo1.setPosition(servoPosition*50);
    }


    @Override
    public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
        Slot0Configs pidConfigs = new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKV(kV)
                .withKA(kA)
                .withKS(kS);
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor1.getConfigurator().apply(pidConfigs));
    }


}
