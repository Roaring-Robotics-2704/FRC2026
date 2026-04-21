// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.hook;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Hook Subsystem. */
public class Hook extends SubsystemBase {
    private final HookIO io;
    private final HookIOInputsAutoLogged inputs = new HookIOInputsAutoLogged();


    private HookState currentState = HookState.UNPOWERED;
    private HookState goalState = HookState.UNPOWERED;

    /** Initializes the subsystem.
     *
     * @param io The IO implementation for the mechanism.
     *
     */
    public Hook(HookIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hook", inputs);

        if (currentState != goalState) {
            switch (goalState) {
                case POWERED:
                    io.setMotorVoltage(Volts.of(6));
                    break;
                case UNPOWERED:
                    io.setMotorVoltage(Volts.of(0));
                    break;
                default:
                    break;
            }
            currentState = goalState; 
        }
    }
    

    /**Sets the desired state for the mechanism.
     *
     *@param desiredState The desired state to set.
     */
    public void setDesiredState(HookState desiredState) {
        goalState = desiredState; 
        
    }

    public boolean isAtDesiredState() {
        return currentState == goalState;
    }

    /**EXAMPLE STATES. */
    public static enum HookState {
        POWERED,
        UNPOWERED,
    }
}
