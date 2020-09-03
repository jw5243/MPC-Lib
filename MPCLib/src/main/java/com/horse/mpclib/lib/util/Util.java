package com.horse.mpclib.lib.util;

import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;

import org.ejml.simple.SimpleMatrix;

public class Util {
    private static final double kEpsilon = 1E-12;

    private Util() {
    }

    public static double limit(double x, double maxMagnitude) {
        return limit(x, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double x, double min, double max) {
        return Math.min(max, Math.max(min, x));
    }

    public static int limit(int x, int min, int max) {
        return Math.min(max, Math.max(min, x));
    }

    public static boolean inRange(double x, double maxMagnitude) {
        return inRange(x, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double x, double min, double max) {
        return x > min && x < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0d, 1d);
        return a + (b - a) * x;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static double getEpsilon() {
        return kEpsilon;
    }

    public static Pose2d convertStateToPose(SimpleMatrix state) {
        return new Pose2d(state.get(0) / 0.0254d, state.get(2) / 0.0254d, new Rotation2d(state.get(4), false));
    }

    public static SimpleMatrix convertPoseToState(Pose2d pose) {
        return new SimpleMatrix(6, 1, false, new double[] {
                pose.getTranslation().x() * 0.0254d, 0d, pose.getTranslation().y() * 0.0254d, 0d, pose.getRotation().getRadians(), 0d
        });
    }
}
