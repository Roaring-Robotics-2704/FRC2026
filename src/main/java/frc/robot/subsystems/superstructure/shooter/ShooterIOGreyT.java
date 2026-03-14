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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
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
import frc.robot.util.OrchestraManager;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ShooterIOGreyT implements ShooterIO {
    private StatusSignal<AngularVelocity> flywheelVelocitySignal;
    private StatusSignal<AngularAcceleration> flywheelAccelerationSignal;
    private StatusSignal<Voltage> flywheelAppliedVoltsSignal;
    private StatusSignal<Current> flywheelCurrentAmpsSignal;
    private StatusSignalCollection statusSignals = new StatusSignalCollection();

    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    private double targetFlywheelVelocity = 0.0;

    /** Instantiates the GreyT Shooter hardware. */
    private LinearServo hoodServo1 = new LinearServo(HOOD_SERVO1_PORT, 50, 6);

    /** Constructs the GreyT shooter code. */
    public ShooterIOGreyT() {
        flywheelMotor1 = new TalonFX(FLYWHEEL_MOTOR_ONE);
        flywheelMotor2 = new TalonFX(FLYWHEEL_MOTOR_TWO);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        Slot0Configs pidConfigs = new Slot0Configs()
                .withKP(SHOOTER_KP)
                .withKD(SHOOTER_KD)
                .withKV(SHOOTER_KV)
                .withKA(SHOOTER_KA)
                .withKS(SHOOTER_KS);
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
        flywheelAppliedVoltsSignal = flywheelMotor1.getMotorVoltage();
        flywheelCurrentAmpsSignal = flywheelMotor1.getStatorCurrent();

        statusSignals.addSignals(
                flywheelVelocitySignal,
                flywheelAccelerationSignal,
                flywheelAppliedVoltsSignal,
                flywheelCurrentAmpsSignal);

        statusSignals.setUpdateFrequencyForAll(Hertz.of(50));
        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor1, flywheelMotor2);
        OrchestraManager.getInstance().addToOrchestra(flywheelMotor1,flywheelMotor2);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        statusSignals.refreshAll();
        inputs.flywheelVelocity.mut_replace(flywheelVelocitySignal.getValue());
        inputs.flywheelAcceleration.mut_replace(flywheelAccelerationSignal.getValue());
        inputs.flywheelAppliedVolts.mut_replace(flywheelAppliedVoltsSignal.getValue());
        inputs.flywheelCurrentAmps.mut_replace(flywheelCurrentAmpsSignal.getValue());

        inputs.hoodAngle.mut_replace(getHoodServoPosition());
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
        flywheelMotor1.setControl(new VelocityDutyCycle(velocity));
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        targetFlywheelVelocity = velocity.in(RotationsPerSecond);

    }

    /**
     * Sets the hood angle by converting the desired angle to servo positions.
     *
     * @param angle
     *            The target angle for the hood.
     */
    @Override
    public void setHoodAngle(Angle angle) {
        double servoPosition = (angle.in(Degrees) - MIN_ANGLE.in(Degrees))
                / (MAX_ANGLE.in(Degrees) - MIN_ANGLE.in(Degrees))*25;
        hoodServo1.setPosition(servoPosition);
    }

    private Angle getHoodServoPosition() {
        return Degrees.of(
           hoodServo1.getPosition() * (MAX_ANGLE.in(Degrees) - MIN_ANGLE.in(Degrees)) + MIN_ANGLE.in(Degrees)*25
        );
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
