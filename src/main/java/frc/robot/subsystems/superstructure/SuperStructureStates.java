// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

/** Add your docs here. */
public class SuperStructureStates {
    public enum WantedState {
        START,//Starting config(All inside robot)
        IDLE, //Default state
        INTAKE, //Intake out
        PASS,  //Gentle shooting with intake stowed
        FEED, //Gentle shooting with intake out
        SHOOT, //Full shooting position
        CLIMB, //Climbing position
    }
    public enum CurrentState {
        START,//Starting config(All inside robot)
        IDLE, //Default state
        INTAKE, //Intake out
        PASS,  //Gentle shooting with intake stowed
        FEED, //Gentle shooting with intake out
        SHOOT, //Full shooting position
        CLIMB, //Climbing position
        TRANSITIONING, //Moving to wanted state
    }
}
