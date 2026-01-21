// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.example;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Example mechanism subsystem using superstructure structuring. */
public class ExampleMech extends SubsystemBase {
    private final ExampleMechIO io; // Initialize a field for the IO, to recieve output commands

    // Ignore "cannot be resolved warning" - this is resolved during project build
    private final ExampleMechIOInputsAutoLogged inputs = new ExampleMechIOInputsAutoLogged();
    // ^ Create the logged inputs instance

    // Current and goal states for the mechanism
    private ExampleMechState currentState = ExampleMechState.STATE1; // Initialize current state to starting state
    private ExampleMechState goalState = ExampleMechState.STATE1; // Initialize goal state to starting state so no movement at start

    /** Initializes the subsystem.
     *
     * @param io The IO implementation for the mechanism.
     *
     */
    public ExampleMech(ExampleMechIO io) {
        this.io = io; // Assign the IO implementation to the field
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs); // Update the inputs from hardware
        Logger.processInputs("Example Mechanism", inputs); // Log the inputs for AdvantageKit

        // State machine logic
        if (currentState != goalState) {
            // Transition to the goal state
            switch (goalState) {
                case STATE1:
                    // Apply outputs for STATE1
                    io.setVoltage(Volts.of(2)/* voltage for STATE1 */);
                    break;
                case STATE2:
                    // Apply outputs for STATE2
                    io.setVoltage(Volts.of(4)/* voltage for STATE2 */);
                    break;
                case STATE3:
                    // Apply outputs for STATE3
                    io.setVoltage(Volts.of(6)/* voltage for STATE3 */);
                    break;
                default:
                    break;
            }
            if (true/* Check if transition to goal state is complete, e.g., based on sensors */) {
                currentState = goalState; // Update current state to goal state
                
            }
        }
    }

    /**Sets the desired state for the mechanism.
     *
     *@param desiredState The desired state to set.
     */
    public void setDesiredState(ExampleMechState desiredState) {
        // Ignore requests to set to transitioning state
        if (desiredState != ExampleMechState.TRANSITIONING) {
            throw new IllegalArgumentException("Cannot set desired state to TRANSITIONING");
        } else if (goalState != desiredState) {
            // Set the desired state for the mechanism
            goalState = desiredState; // Update goal state
            currentState = ExampleMechState.TRANSITIONING; // Set current state to transitioning
        
        }
    }

    public boolean isAtDesiredState() {
        return currentState == goalState;
    }

    /**EXAMPLE STATES. */
    public static enum ExampleMechState {
        STATE1, // Example state 1
        STATE2, // Example state 2
        STATE3, // Example state 3
        TRANSITIONING, // NEEDED THIS STATE FOR SUPERSTRUCTURE
    }
}
