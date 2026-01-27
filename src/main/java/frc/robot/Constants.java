// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    /**
     * Runtime mode selector for the robot application.
     *
     * <p>Determines how the system should behave with respect to hardware, timing,
     * and data sources:
     * <ul>
     *   <li>{@link #REAL} — Running on the physical robot; use real hardware and sensors.</li>
     *   <li>{@link #SIM} — Running in a physics simulator; substitute simulated
     *       sensors/actuators and physics-driven timing.</li>
     *   <li>{@link #REPLAY} — Replaying from recorded logs; use recorded sensor inputs
     *       and deterministic playback semantics.</li>
     * </ul>
     *
     * <p>Use this enum to branch initialization and runtime logic so that code paths
     * for hardware access, logging, and timing are appropriate for the current execution context.
     */
    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
    
    
}
