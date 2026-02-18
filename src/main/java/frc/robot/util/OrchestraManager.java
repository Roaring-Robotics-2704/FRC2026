// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class OrchestraManager {
    private Orchestra orchestra;
    private static OrchestraManager orchestraManager;

    private OrchestraManager() {
        orchestra = new Orchestra();
    }
    /** Returns the singleton instance of the OrchestraManager. */
    public static OrchestraManager getInstance() {
        if (orchestraManager == null) {
            orchestraManager = new OrchestraManager();
        }
        return orchestraManager;
    }

    /** Adds instruments to the orchestra. */
    public void addToOrchestra(TalonFX... objects) {
        for (TalonFX obj : objects) {
            orchestra.addInstrument(obj);
        }
    }
    
    /** Plays the orchestra. */
    public void play() {
        if (!orchestra.isPlaying()) {
            orchestra.play();
        }
    }

    /** Stops the orchestra. */
    public void stop() {
        if (orchestra.isPlaying()) {
            orchestra.stop();
        }
    }

    public void loadFile(String songName) {
        orchestra.loadMusic(Filesystem.getDeployDirectory() + "/music/" + songName + ".chrp");
    }

}
