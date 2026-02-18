// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.solvers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class BasicTunedCalc implements SolverIO {
    private static BasicTunedCalc instance;
    private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    private int rotationsPerMinute = 2800;

    public BasicTunedCalc() {
        // Example data points (distance in meters, angle in degrees)
        hoodAngleMap.put(1.0, 10.0);
        hoodAngleMap.put(2.0, 20.0);
        hoodAngleMap.put(3.0, 30.0);
        hoodAngleMap.put(4.0, 40.0);
        hoodAngleMap.put(5.0, 45.0);
    }


    // @Override
    // public ShootingSolution getShootingSolution(Pose2d robotPose) {
    //     Distance distance = Meters.of(robotPose.getTranslation());
    //     Angle hoodAngle = getHoodAngle(distance);
    //     return new ShootingSolution(DegreesPerSecond.of(rotationsPerMinute * 360.0 / 60.0), hoodAngle, Rotation2d.kZero);
    // }
}
