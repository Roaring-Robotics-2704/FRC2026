package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MOTOR_TYPE;

import java.rmi.server.RMIClassLoader;

import org.dyn4j.geometry.Rectangle;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.*;

public class IntakeIOSim implements IntakeIO {

    LinearSystem<N2, N1, N2> slideSystem = LinearSystemId.createElevatorSystem(SLIDE_MOTOR_TYPE, 0, 0, 0);
    MapleMotorSim slideMotorSim = new MapleMotorSim(
        new SimMotorConfigs(SLIDE_MOTOR_TYPE, MOTOR_GEAR_RATIO, KilogramSquareMeters.of(0.002), Volts.of(0.2)));
    MapleMotorSim rollerMotorSim = new MapleMotorSim(
        new SimMotorConfigs(ROLLER_MOTOR_TYPE, 2, KilogramSquareMeters.of(0.002), Volts.of(0.2)));

    ElevatorSim slideSim = new ElevatorSim(slideSystem, SLIDE_MOTOR_TYPE, 0, SLIDE_MAX_DISTANCE.in(Meters), false, 0);

    IntakeSimulation intakeSim;
    /** Instantiates the Real Intake hardware. */
    public IntakeIOSim() {
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerVelocity.mut_replace(
            slideMotorSim.getEncoderVelocity());
        inputs.slideAppliedVoltage.mut_replace(slideMotorSim.getAppliedVoltage());
    }

    @Override
    public void setSlideVoltage(Voltage voltage) {
        slideMotorSim.useSimpleDCMotorController().requestVoltage(voltage);
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotorSim.useSimpleDCMotorController().requestVoltage(voltage);
    }

    @Override
    public void setPosition(Distance position) {
        slideMotorSim.useSimpleDCMotorController().
    }
    
    @Override
    public void stopMotors() {
        slideMotorSim.useSimpleDCMotorController().requestVoltage(Volts.of(0));
        rollerMotorSim.useSimpleDCMotorController().requestVoltage(Volts.of(0));
    }

    @Override
    public void resetSlideEncoder(Distance position) {
        slideMotorSim.setEncoderPosition(position.in(Meters));
        
    }
}
