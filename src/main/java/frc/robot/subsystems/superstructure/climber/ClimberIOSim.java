// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.climber.ClimberConstants.CLIMBER_GEARING;
import static frc.robot.subsystems.superstructure.climber.ClimberConstants.CLIMBER_MOTOR_TYPE;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberIOSim implements ClimberIO {
    // MapleMotorSim climberMotorSim =
    // new MapleMotorSim(new SimMotorConfigs(CLIMBER_MOTOR_TYPE, CLIMBER_GEARING, ,
    // Volts.of(0.5)));

    public ClimberIOSim() {
    }
}
