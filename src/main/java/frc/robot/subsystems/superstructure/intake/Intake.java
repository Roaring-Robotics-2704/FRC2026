// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperStructure;

public class Intake extends SubsystemBase {


  /** Creates a new Intake. */
  public Intake(SuperStructure superStructure) {

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  // Command section

  /**
   * Deploys the intake
   * @return command to deploy the intake
   */
  public Command deployIntake() {
    // Return command to deploy intake
    return null;
  }

  /**
   * Stows the intake
   * @return command to stow the intake
   */
  public Command stowIntake() {
    // Return command to stow intake
    return null;
  }

  /**
   * Activates the intake motors
   * @param reversed if true, runs intake in reverse
   * @return command to run intake motors
   */
  public Command activateIntake(boolean reversed) {
    // Return command to run intake motors
    return null;
  }

  /**
   * Deactivates the intake motors
   * @return command to stop intake motors
   */
  public Command deactivateIntake() {
    // Return command to stop intake motors
    return null;
  }

}
