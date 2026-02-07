package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.ENCODER_GEAR_RATIO;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.ROLLER_CURRENT_LIMIT;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_CURRENT_LIMIT;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MAX_ACCELERATION;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MAX_VELOCITY;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KA;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KD;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KG;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KI;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KP;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KS;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_POSITION_KV;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.ROLLER_MOTOR_TYPE;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MOTOR_TYPE;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.SparkUtil;

public class IntakeIOSim implements IntakeIO {
    private SparkFlex rollerMotor;
    private SparkFlexSim rollerMotorSim;
    private SparkMax slideMotor;
    private SparkMaxSim slideMotorSim;
    private SparkMaxConfig slideConfig;

    /** Instantiates the Real Intake hardware. */
    public IntakeIOSim() {
        rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        slideMotor = new SparkMax(IntakeConstants.SLIDE_MOTOR_ID, MotorType.kBrushless);
        slideConfig = new SparkMaxConfig();

        slideConfig.smartCurrentLimit(SLIDE_CURRENT_LIMIT);
        slideConfig.closedLoop.feedForward.kG(SLIDE_POSITION_KG);
        slideConfig.closedLoop.feedForward.kS(SLIDE_POSITION_KS);
        slideConfig.closedLoop.feedForward.kV(SLIDE_POSITION_KV);
        slideConfig.closedLoop.feedForward.kA(SLIDE_POSITION_KA);
        slideConfig.closedLoop.pid(SLIDE_POSITION_KP, SLIDE_POSITION_KI, SLIDE_POSITION_KD);

        slideConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        slideConfig.absoluteEncoder.inverted(false);
        slideConfig.absoluteEncoder.positionConversionFactor(ENCODER_GEAR_RATIO);


        slideConfig.closedLoop.maxMotion.maxAcceleration(SLIDE_MAX_ACCELERATION);
        slideConfig.closedLoop.maxMotion.cruiseVelocity(SLIDE_MAX_VELOCITY);
        SparkUtil.tryUntilOk(slideMotor, 5,
                () -> slideMotor.configure(slideConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerConfig.idleMode(IdleMode.kCoast);
        SparkUtil.tryUntilOk(rollerMotor, 5,
                () -> rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        slideMotorSim = new SparkMaxSim(slideMotor, SLIDE_MOTOR_TYPE);
        rollerMotorSim = new SparkFlexSim(rollerMotor, ROLLER_MOTOR_TYPE);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerMotorSim.iterate(rollerMotorSim.getVelocity(), 12, 0.02);
        slideMotorSim.iterate(slideMotorSim.getVelocity(), 12, 0.02);
    
        inputs.slideAppliedVoltage.mut_replace(slideMotorSim.getAppliedOutput(), Volts);
        inputs.slideCurrentDraw.mut_replace(slideMotorSim.getMotorCurrent(), Amps);
        inputs.slidePosition.mut_replace(slideMotorSim.getPosition(), Inches);
        inputs.slideAtPosition = slideMotor.getClosedLoopController().isAtSetpoint();
        inputs.slideVelocity.mut_replace(slideMotorSim.getVelocity(), InchesPerSecond);

        inputs.rollerAppliedVoltage.mut_replace(rollerMotorSim.getAppliedOutput(), Volts);
        inputs.rollerCurrentDraw.mut_replace(rollerMotorSim.getMotorCurrent(), Amps);
        inputs.rollerVelocity.mut_replace(rollerMotorSim.getVelocity(), RadiansPerSecond);
    }

    @Override
    public void setSlideVoltage(Voltage voltage) {
        slideMotor.setVoltage(voltage);
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void setPosition(Distance position) {
        slideMotor.getClosedLoopController().setSetpoint(position.in(Inches), ControlType.kMAXMotionPositionControl);
    }
    
    @Override
    public void stopMotors() {
        slideMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    @Override
    public void resetSlideEncoder(Distance position) {
        slideMotor.getEncoder().setPosition(position.in(Inches));
    }
}
