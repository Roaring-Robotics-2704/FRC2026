// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.*;

/** Add your docs here. */
public class ShooterIOLQR implements ShooterIO {
    private StatusSignal<AngularVelocity> leftflywheelVelocitySignal;
    private StatusSignal<AngularVelocity> rightflywheelVelocitySignal;
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


    // Volts per (radian per second)
    private static final double kFlywheelKv = 0.01822;

    // Volts per (radian per second squared)
    private static final double kFlywheelKa = 0.0014421;

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> flywheelPlant =
        LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

    private final KalmanFilter<N1, N1, N1> observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder
            // data is
            Constants.loopTimeSeconds);

    private final LinearQuadraticRegulator<N1, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            flywheelPlant,
            VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            Constants.loopTimeSeconds); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(flywheelPlant, controller, observer, 10.0, Constants.loopTimeSeconds);

    /** Instantiates the GreyT Shooter hardware. */
    private LinearServo hoodServo1 = new LinearServo(HOOD_SERVO1_PORT,50, 6);

    /** Constructs the GreyT shooter code. */
    public ShooterIOLQR() {
        flywheelMotor1 = new TalonFX(FLYWHEEL_MOTOR_ONE);
        flywheelMotor2 = new TalonFX(FLYWHEEL_MOTOR_TWO);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive);
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withStatorCurrentLimitEnable(true);
        // in init function


        AudioConfigs audioConfigs = new AudioConfigs().withAllowMusicDurDisable(true).withBeepOnConfig(true).withBeepOnBoot(true);

        TalonFXConfiguration config = new TalonFXConfiguration()
                .withMotorOutput(
                        motorOutput)
                .withCurrentLimits(
                        currentLimits)
            .withAudio(audioConfigs);

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor1.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor2.getConfigurator().apply(config));

        leftflywheelVelocitySignal = flywheelMotor1.getVelocity();
        rightflywheelVelocitySignal = flywheelMotor2.getVelocity();
        flywheelAccelerationSignal = flywheelMotor1.getAcceleration();
        leftFlywheelAppliedVoltsSignal = flywheelMotor1.getMotorVoltage();
        rightFlyWheelAppliedVoltsSignal = flywheelMotor2.getMotorVoltage();
        leftFlywheelCurrentAmpsSignal = flywheelMotor1.getStatorCurrent();
        rightFlywheelCurrentAmpsSignal = flywheelMotor2.getStatorCurrent();
        leftTempSignal = flywheelMotor1.getDeviceTemp();
        rightTempSignal = flywheelMotor2.getDeviceTemp();
       statusSignals.addSignals(
                leftflywheelVelocitySignal,
                rightflywheelVelocitySignal,
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
        inputs.flywheelVelocity.mut_replace(leftflywheelVelocitySignal.getValue());
        inputs.flywheelAcceleration.mut_replace(flywheelAccelerationSignal.getValue());
        inputs.leftFlywheelAppliedVolts.mut_replace(leftFlywheelAppliedVoltsSignal.getValue());
        inputs.rightFlywheelAppliedVolts.mut_replace(rightFlyWheelAppliedVoltsSignal.getValue());
        inputs.leftFlywheelCurrentAmps.mut_replace(leftFlywheelCurrentAmpsSignal.getValue());
        inputs.rightFlywheelCurrentAmps.mut_replace(rightFlywheelCurrentAmpsSignal.getValue());

        inputs.leftVelocity.mut_replace(leftflywheelVelocitySignal.getValue());
        inputs.rightVelocity.mut_replace(rightflywheelVelocitySignal.getValue());
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
        m_loop.setNextR(VecBuilder.fill(velocity.in(RadiansPerSecond)));
        m_loop.correct(VecBuilder.fill(leftflywheelVelocitySignal.getValue().in(RadiansPerSecond)));
        m_loop.predict(Constants.loopTimeSeconds);
        double nextVoltage = m_loop.getU(0);
        flywheelMotor1.setControl(new VoltageOut(Volts.of(nextVoltage)));
    }

    @Override
    public void setFlywheelVoltage(Voltage voltage) {
        flywheelMotor1.setControl(new VoltageOut(voltage));
        flywheelMotor2.setControl(new VoltageOut(voltage.unaryMinus()));
    }
    /**
     * Sets the hood angle by converting the desired angle to servo positions.
     *
     * @param percent
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
        throw new UnsupportedOperationException("This method is not supported for LQR.");
    }


}
