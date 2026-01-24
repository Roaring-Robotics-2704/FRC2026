// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.MAX_SHOOTER_VOLTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  boolean manual = false;

  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
 
  private double median = 0;
  private double medianCount = 0; // median filters window of data for little offset errors
  private boolean hasRanCalibration = false;

  MedianFilter filter = new MedianFilter(10); // adjust this value later if necessary

  private ProfiledPIDController controller = new ProfiledPIDController(
    ShooterConstants.SHOOTER_KP, 
    ShooterConstants.SHOOTER_KI, 
    ShooterConstants.SHOOTER_KD, 
    new TrapezoidProfile.Constraints(1, 10)); // adjust the trapezoid profile values as needed

  private SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.of(0.5).per(Second),
      Volts.of(2),
      null,
      state -> Logger.recordOutput("Elevator.SysIdState", state.toString())),
    new SysIdRoutine.Mechanism(this::setElevatorVolts, null, local()));

  public Shooter(ShooterIO io, RobotContainer robotContainer) {
    SmartDashboard.putData("Elevator PID", controller);
    this.io = io;
    this.robotContainer = robotContainer;
    io.init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  private Shooter local() {
    return this;
  }
}
