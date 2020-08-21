package org.firstinspires.ftc.teamcode.lib.motion;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.IntStream;

public class SplineGenerator {
    public static Spline getInitialSpline(int degree, Pose2d initialPose, Translation2d terminationPoint, double... values) {
        if(values.length % 2 == 0) {
            return null;
        }

        if(initialPose.getRotation().cos() >= 0) {
            values[0] = Math.abs(values[0]);
        } else {
            values[0] = -Math.abs(values[0]);
        }

        List<Double> coefficients = new LinkedList<>(Arrays.asList(initialPose.getTranslation().x(), initialPose.getTranslation().y(), //Constant term
                values[0], values[0] * initialPose.getRotation().tan(), //Linear term
                2d * (terminationPoint.x() - initialPose.getTranslation().x() - values[0] - IntStream //Quadratic term
                        .range(1, values.length)
                        .filter(n -> n % 2 == 1)
                        .mapToDouble(n -> values[n] / factorial((n + 5) / 2))
                        .sum()),
                2d * (terminationPoint.y() - initialPose.getTranslation().y() - values[0] * initialPose.getRotation().tan() - IntStream
                        .range(2, values.length)
                        .filter(n -> n % 2 == 0)
                        .mapToDouble(n -> values[n] / factorial((n + 4) / 2))
                        .sum())
        ));

        for(int i = 1; i < values.length; i++) {
            coefficients.add(values[i]);
        }

        return new Spline(degree, coefficients.stream().mapToDouble(Double::doubleValue).toArray(), false);
    }

    public static Spline getTerminatingSpline(int degree, Spline previousSpline, Rotation2d initialHeading, Translation2d previousPoint, Pose2d terminationPose, double... values) {
        if(values.length % 2 == 0 && degree == previousSpline.getPolynomialDegree()) {
            return null;
        }

        if(terminationPose.getRotation().cos() >= 0) {
            values[0] = -Math.abs(values[0]);
        } else {
            values[0] = Math.abs(values[0]);
        }

        List<Double> coefficients = new LinkedList<>(Arrays.asList(terminationPose.getTranslation().x(), terminationPose.getTranslation().y(), //Constant term
                values[0], values[0] * terminationPose.getRotation().tan(), //Linear term
                2d * (-3d * terminationPose.getTranslation().x() + 5d * previousPoint.x() - 2d * previousSpline.getCoefficient(0, Spline.Axis.X) - //Quadratic term
                        (previousSpline.getCoefficient(1, Spline.Axis.X) + 2 * values[0]) + IntStream
                        .range(1, values.length + 2)
                        .filter(n -> n % 2 == 1)
                        .mapToDouble(n -> (((n + 1) / 2) * previousSpline.getCoefficients()[n + 5] + ((n - 1) / 2) * (n == 1 ? 0 : values[n - 2])) / factorial((n + 5) / 2))
                        .sum()),
                2d * (-3d * terminationPose.getTranslation().y() + /*3d*/5d * previousPoint.y() - 2d * previousSpline.getCoefficient(0, Spline.Axis.X) -
                        (previousSpline.getCoefficient(1, Spline.Axis.X) * initialHeading.tan() + 2 * values[0] * terminationPose.getRotation().tan()) + IntStream
                        .range(2, values.length + 2)
                        .filter(n -> n % 2 == 0)
                        .mapToDouble(n -> ((n / 2) * previousSpline.getCoefficients()[n + 5] + ((n - 2) / 2) * (n == 2 ? 0 : values[n - 2])) / factorial((n + 4) / 2))
                        .sum()),
                6d * (2d * terminationPose.getTranslation().x() - 4d * previousPoint.x() + 2d * previousSpline.getCoefficient(0, Spline.Axis.X) - //Cubic term
                        previousSpline.getCoefficient(3, Spline.Axis.X) / 6d + (previousSpline.getCoefficient(1, Spline.Axis.X) + values[0]) - IntStream
                        .range(3, values.length + 2)
                        .filter(n -> n % 2 == 1)
                        .mapToDouble(n -> ((n + 1) / 2) * (previousSpline.getCoefficient((n + 5) / 2, Spline.Axis.X) + values[n - 2]) / factorial((n + 5) / 2))
                        .sum()),
                6d * (2d * terminationPose.getTranslation().y() - /*2d*/4d * previousPoint.y() + 2d * previousSpline.getCoefficient(0, Spline.Axis.Y) -
                        previousSpline.getCoefficient(3, Spline.Axis.Y) / 6d + (previousSpline.getCoefficient(1, Spline.Axis.X) * initialHeading.tan() +
                        values[0] * terminationPose.getRotation().tan()) - IntStream
                        .range(4, values.length + 2)
                        .filter(n -> n % 2 == 0)
                        .mapToDouble(n -> (n / 2) * (previousSpline.getCoefficient((n + 4) / 2, Spline.Axis.Y) + values[n - 2]) / factorial((n + 4) / 2))
                        .sum())
        ));

        for(int i = 1; i < values.length; i++) {
            coefficients.add(values[i]);
        }

        return new Spline(degree, coefficients.stream().mapToDouble(Double::doubleValue).toArray(), true);
    }

    public static void main(String... args) {
        Spline spline = SplineGenerator.getInitialSpline(4, new Pose2d(0, 0, new Rotation2d(Math.PI / 2.1d, false)),
                new Translation2d(5d, 5d), 1, 1, 1, 1, 1);
        Spline finalSpline = SplineGenerator.getTerminatingSpline(4, spline, new Rotation2d(Math.PI / 2.1d, false),
                new Translation2d(5d, 5d), new Pose2d(10d, 10d, new Rotation2d(Math.PI - Math.PI / 2.1d, false)), -1, 1, 1);

        System.out.println(spline);
        System.out.println(finalSpline);
        System.out.println(spline.getMeanCurvature());
        System.out.println(spline.getMeanDCurvature());
        System.out.println(spline.getArcLength());
        System.out.println(spline.getMinDistanceFromPoint(new Translation2d(6, 6)));
    }

    private static int factorial(int value, int result) {
        return value <= 1 ? result : factorial(value - 1, result * (value - 1));
    }

    private static int factorial(int value) {
        return value <= 1 ? 1 : factorial(value, value);
    }
}
