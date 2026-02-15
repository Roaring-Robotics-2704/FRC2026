package frc.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.intake.IntakeConstants.SLIDE_MOTOR_TYPE;

import java.rmi.server.RMIClassLoader;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeIOSim implements IntakeIO {

    LinearSystem<N2, N1, N2> slideSystem = LinearSystemId.createElevatorSystem(SLIDE_MOTOR_TYPE, 0, 0, 0);
    MapleMotorSim slideMotorSim = new MapleMotorSim(new SimMotorConfigs(SLIDE_MOTOR_TYPE, 0, null, null));

    Voltage rollerVoltage = Volts.of(0);

    /** Instantiates the Real Intake hardware. */
    public IntakeIOSim() {
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerAppliedVoltage.mut_replace(rollerVoltage);
        inputs.rollerVelocity.mut_replace(
            IntakeConstants.ROLLER_MOTOR_TYPE.freeSpeedRadPerSec * (rollerVoltage.in(Volts) / 12), RadiansPerSecond);
    }

    @Override
    public void setSlideVoltage(Voltage voltage) {
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerVoltage = voltage;
    }

    @Override
    public void setPosition(Distance position) {
    }
    
    @Override
    public void stopMotors() {
   
    }

    @Override
    public void resetSlideEncoder(Distance position) {
    }
}
