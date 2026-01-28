// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class HoodAngleCalc {
    private static HoodAngleCalc instance;
    private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    private HoodAngleCalc() {
        // Example data points (distance in meters, angle in degrees)
        hoodAngleMap.put(1.0, 10.0);
        hoodAngleMap.put(2.0, 20.0);
        hoodAngleMap.put(3.0, 30.0);
        hoodAngleMap.put(4.0, 40.0);
        hoodAngleMap.put(5.0, 45.0);
    }

    /** Singleton pattern to get the instance of HoodAngleCalc. */
    public static HoodAngleCalc getInstance() {
        if (instance == null) {
            instance = new HoodAngleCalc();
        }
        return instance;
    }

    public Angle getHoodAngle(Distance distance) {
        return Degrees.of(hoodAngleMap.get(distance.in(Meters)));
    }
}
