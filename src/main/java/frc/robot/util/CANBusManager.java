// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.CAN;

/** Add your docs here. */
public class CANBusManager {
    private static CANBusManager instance;

    private CANBus rioBus = CANBus.roboRIO();
    private CANBus driveBus = new CANBus("Drivetrain");
    
    private CANBusManager() {
    }

    /** Get the singleton instance of the CANBusManager. */
    public static CANBusManager getInstance() {
        if (instance == null) {
            instance = new CANBusManager();
        }
        return instance;
    }

    public CANBus getRIOBus() {
        return rioBus;
    }

    public CANBus getDriveBus() {
        return driveBus;
    }
    
}
