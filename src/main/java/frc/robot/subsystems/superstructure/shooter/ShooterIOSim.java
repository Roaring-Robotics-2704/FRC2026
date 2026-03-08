package frc.robot.subsystems.superstructure.shooter;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KA;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KD;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KP;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KS;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_KV;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.SHOOTER_MOTOR_TYPE;
import frc.robot.util.simUtils.SimMotorConfigs;
import frc.robot.util.simUtils.SimulatedMotor;

public class ShooterIOSim implements ShooterIO {
    Angle hoodAngle = Degrees.zero();
    AngularVelocity flywheelVelocity = RotationsPerSecond.zero();
    LinearSystem<N1, N1, N1> flywheelSystem = LinearSystemId.createFlywheelSystem(SHOOTER_MOTOR_TYPE, 0.001, 1);
    SimulatedMotor flywheelMotorSim = new SimulatedMotor(
        new SimMotorConfigs(SHOOTER_MOTOR_TYPE, 1, KilogramSquareMeters.of(0.001), Volts.of(0.5)));
    private FlywheelSim flywheelSim = new FlywheelSim(flywheelSystem, SHOOTER_MOTOR_TYPE, 0.02);
    private PIDController flywheelPID = new PIDController(SHOOTER_KP, 0, SHOOTER_KD);

    private SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(SHOOTER_KS, SHOOTER_KV, SHOOTER_KA);



    public ShooterIOSim() {
        super();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.setInputVoltage(flywheelMotorSim.getAppliedVoltage().in(Volts));
        flywheelSim.update(0.02);
        flywheelMotorSim.update(Seconds.of(0.02));
        inputs.hoodAngle.mut_replace(hoodAngle);
        inputs.flywheelVelocity.mut_replace(RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60));
        inputs.flywheelAppliedVolts.mut_replace(Volts.of(flywheelSim.getInputVoltage()));
        inputs.atTargetAngle = true; // No hood in simulation, so always at target angle
        inputs.atTargetVelocity = true;
        // inputs.atTargetVelocity = MathUtil.isNear(
        //     flywheelSim.getAngularVelocityRPM(),
        //     flywheelVelocity.in(RotationsPerSecond) / 60,
        //     50); // 50 RPM tolerance
        inputs.flywheelAcceleration.mut_replace(
            RotationsPerSecondPerSecond.of(flywheelSim.getAngularAccelerationRadPerSecSq()));
        inputs.flywheelCurrentAmps.mut_replace(flywheelSim.getCurrentDrawAmps(), Amps);
        inputs.targetFlywheelVelocity.mut_replace(flywheelVelocity);
    }

    @Override
    public void setFlywheelVoltage(Voltage volts) {
        flywheelMotorSim.useSimpleDCMotorController().requestVoltage(volts);

    }

    @Override
    public void setHoodAngle(Angle angle) {
        // No hood in simulation
        hoodAngle = angle;
    }

    @Override 
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelVelocity = velocity;
        flywheelPID.setSetpoint(velocity.in(RotationsPerSecond));
        double currentSpeed = flywheelSim.getAngularVelocityRPM() / 60; // Convert RPM to RPS
        
        flywheelMotorSim.useSimpleDCMotorController().requestVoltage(
            Volts.of(
                flywheelPID.calculate(currentSpeed)
                + flywheelFeedforward.calculateWithVelocities(currentSpeed, velocity.in(RotationsPerSecond))));
    }
    

}