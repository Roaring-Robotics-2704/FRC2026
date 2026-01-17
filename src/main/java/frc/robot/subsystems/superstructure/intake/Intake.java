// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

/** Intake subsystem for controlling the robot's intake mechanism. */
public class Intake extends SubsystemBase {
    private Goal goal = Goal.STOWED;

    /** Creates a new Intake. */
    public Intake() {
    }

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        switch (goal) {
            case STOWED:
                //
                break;

            default:
                break;
        }
    }

    // Command section

    /**
     * Deploys the intake.
     *
     * @param motorOn whether the intake motor should be on
     * @return command to deploy the intake
     */
    public Command deployIntake(BooleanSupplier motorOn) {
        // Return command to deploy intake
        // Should deploy intake and turn off brake mode
        return Commands.either(
                Commands.run(() -> goal = Goal.DEPLOYED_ON, this),
                Commands.run(() -> goal = Goal.DEPLOYED_OFF, this),
                motorOn);
    }

    /**
     * Stows the intake.
     *
     * @return command to stow the intake
     */
    public Command stowIntake() {
        // Return command to stow intake
        // Should stow intake and turn on brake mode
        return Commands.run(() -> goal = Goal.STOWED, this);
    }

    /** Possible goals for the intake subsystem. */
    public enum Goal {
        STOWED,
        DEPLOYED_OFF,
        DEPLOYED_ON
    }
}
