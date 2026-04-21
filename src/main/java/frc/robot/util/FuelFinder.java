// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class FuelFinder {
    private static double estimateBandwidth(List<Pose2d> balls) {
        if (balls.size() < 2)
            return 0.5; // fallback

        double totalNearest = 0;

        for (Pose2d a : balls) {
            double minDist = Double.MAX_VALUE;

            for (Pose2d b : balls) {
                if (a == b)
                    continue;
                double d = Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
                minDist = Math.min(minDist, d);
            }

            totalNearest += minDist;
        }

        return totalNearest / balls.size();
    }

    /** Finds the densest cluster of balls. Uses mean shift clustering with a Gaussian kernel. */
    public static Pose2d findDensestCluster(List<Pose2d> balls) {
        if (balls.isEmpty())
            return null;

        double bandwidth = estimateBandwidth(balls);
        double bandwidthSquared = bandwidth * bandwidth;

        List<double[]> clusterCenters = new ArrayList<>();

        for (Pose2d start : balls) {
            double x = start.getX();
            double y = start.getY();

            for (int iter = 0; iter < 20; iter++) { // converge
                double sumX = 0;
                double sumY = 0;
                double weightSum = 0;

                for (Pose2d p : balls) {
                    double dx = x - p.getX();
                    double dy = y - p.getY();
                    double distSq = dx * dx + dy * dy;

                    double weight = Math.exp(-distSq / (2 * bandwidthSquared));
                    sumX += p.getX() * weight;
                    sumY += p.getY() * weight;
                    weightSum += weight;
                }

                double newX = sumX / weightSum;
                double newY = sumY / weightSum;

                if (Math.hypot(newX - x, newY - y) < 1e-3)
                    break;

                x = newX;
                y = newY;
            }

            clusterCenters.add(new double[] { x, y });
        }

        // Count convergence density
        Map<String, Integer> counts = new HashMap<>();

        for (double[] c : clusterCenters) {
            int keyX = (int) (c[0] * 100); // round
            int keyY = (int) (c[1] * 100);
            String key = keyX + "," + keyY;
            counts.put(key, counts.getOrDefault(key, 0) + 1);
        }

        String maxKey = null;
        int maxCount = 0;

        for (Map.Entry<String, Integer> entry : counts.entrySet()) {
            if (entry.getValue() > maxCount) {
                maxCount = entry.getValue();
                maxKey = entry.getKey();
            }
        }

        if (maxKey == null)
            return null;

        String[] parts = maxKey.split(",");
        double finalX = Integer.parseInt(parts[0]) / 100.0;
        double finalY = Integer.parseInt(parts[1]) / 100.0;

        return new Pose2d(finalX, finalY, new Rotation2d());
    }
}
