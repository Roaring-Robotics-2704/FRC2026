// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
        System.out.println("Loading music file: " + Filesystem.getDeployDirectory() + "/music/" + songName + ".chrp");
        //Print if file exists
        if (Filesystem.getDeployDirectory() == null) {
            System.out.println("Deploy directory is null!");
        } else {
            java.io.File musicFile = new java.io.File(Filesystem.getDeployDirectory() + "/music/" + songName + ".chrp");
            if (musicFile.exists()) {
                System.out.println("Music file found: " + musicFile.getAbsolutePath());
            } else {
                System.out.println("Music file not found: " + musicFile.getAbsolutePath());
            }
        }
        orchestra.loadMusic(Filesystem.getDeployDirectory() + "/music/" + songName + ".chrp");
    }

    public Command playOrchestraCommand(String songName) {
        return Commands.sequence(
            Commands.runOnce(() -> loadFile(songName)).ignoringDisable(true),
            Commands.runOnce(this::play).ignoringDisable(true),
            Commands.waitUntil(() -> !orchestra.isPlaying()).ignoringDisable(true)
        ).ignoringDisable(true);
    }
}
