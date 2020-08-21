package com.horse.mpclib.lib.motion;

import com.horse.mpclib.lib.geometry.Translation2d;

import java.util.stream.IntStream;

public class Spline implements ParametricFunction {
    private static final int PARAMETER_STEPS = 2000;

    private int polynomialDegree;
    private double[] coefficients;
    private boolean inverted;

    public enum Axis {
        X(0), Y(1);

        Axis(int offset) {
            this.offset = offset;
        }

        private final int offset;

        public int getOffset() {
            return offset;
        }
    }

    public Spline(int polynomialDegree) {
        this(polynomialDegree, false);
    }

    public Spline(int polynomialDegree, boolean inverted) {
        this(polynomialDegree, new double[2 * (polynomialDegree + 1)], inverted);
    }

    public Spline(int polynomialDegree, double[] coefficients, boolean inverted) {
        if(polynomialDegree == coefficients.length / 2 - 1) {
            setPolynomialDegree(polynomialDegree);
            setCoefficients(coefficients);
            setInverted(inverted);
        } else {
            System.out.println("Invalid parameter input for spline constructor. Given degree " + polynomialDegree + " but found " + (coefficients.length / 2) + " degree from data.");
        }
    }

    public static void main(String... args) {
        Spline spline = new Spline(2, new double[] {
                0, 0, 1, 0, 0, 1
        }, false);

        int steps = 100;
        for(int i = 0; i < steps; i++) {
            double val = (double)(i) / steps;
            System.out.println(val + "\t" + spline.evaluate(val) + "\t" + Math.pow(val, 2d) / 2d);
        }
    }

    public double getCoefficient(int order, Axis axis) {
        return getCoefficients()[2 * order + axis.getOffset()];
    }

    private int factorial(int value, int result) {
        return value <= 1 ? result : factorial(value - 1, result * (value - 1));
    }

    private int factorial(int value) {
        return value <= 1 ? 1 : factorial(value, value);
    }

    @Override
    public String toString() {
        StringBuilder stringBuilder = new StringBuilder("{");
        for(int i = 0; i < getCoefficients().length; i += 2) {
            stringBuilder.append("{").append(getCoefficients()[i]).append(",").append(getCoefficients()[i + 1]).append((i + 2 < getCoefficients().length) ? "}," : "}");
        }

        stringBuilder.append("}");
        return stringBuilder.toString();
    }

    @Override
    public Translation2d evaluate(double parameter) {
        Translation2d vector = new Translation2d();
        for(int i = 0; i <= getPolynomialDegree(); i++) {
            vector = vector.translateBy(new Translation2d(getCoefficients()[2 * i], getCoefficients()[2 * i + 1])
                    .scale(Math.pow(isInverted() ? (1 - parameter) : parameter, i) / factorial(i)));
        }

        return vector;
    }

    @Override
    public Translation2d getDerivative(double parameter) {
        Translation2d vector = new Translation2d();
        for(int i = 1; i <= getPolynomialDegree(); i++) {
            vector = vector.translateBy(new Translation2d(getCoefficients()[2 * i], getCoefficients()[2 * i + 1])
                    .scale((isInverted() ? -1d : 1d) * Math.pow(isInverted() ? (1 - parameter) : parameter, i - 1) / factorial(i - 1)));
        }

        return vector;
    }

    @Override
    public Translation2d getSecondDerivative(double parameter) {
        Translation2d vector = new Translation2d();
        for(int i = 2; i <= getPolynomialDegree(); i++) {
            vector = vector.translateBy(new Translation2d(getCoefficients()[2 * i], getCoefficients()[2 * i + 1])
                    .scale(Math.pow(isInverted() ? (1 - parameter) : parameter, i - 2) / factorial(i - 2)));
        }

        return vector;
    }

    public Translation2d getThirdDerivative(double parameter) {
        Translation2d vector = new Translation2d();
        for(int i = 3; i <= getPolynomialDegree(); i++) {
            vector = vector.translateBy(new Translation2d(getCoefficients()[2 * i], getCoefficients()[2 * i + 1])
                    .scale((isInverted() ? -1d : 1d) * Math.pow(isInverted() ? (1 - parameter) : parameter, i - 3) / factorial(i - 3)));
        }

        return vector;
    }

    @Override
    public double getCurvature(double parameter) {
        Translation2d derivative = getDerivative(parameter);
        Translation2d secondDerivative = getSecondDerivative(parameter);
        return Math.abs(derivative.x() * secondDerivative.y() - secondDerivative.x() * derivative.y()) / Math.pow(derivative.norm(), 3d);
    }

    @Override
    public double getDCurvature(double parameter) {
        Translation2d derivative = getDerivative(parameter);
        Translation2d secondDerivative = getSecondDerivative(parameter);
        Translation2d thirdDerivative = getThirdDerivative(parameter);
        return Math.abs(6d * (derivative.y() * secondDerivative.x() - secondDerivative.y() * derivative.x())
                * (derivative.x() * secondDerivative.x() + derivative.y() * secondDerivative.y()) +
                2d * derivative.norm2() * (derivative.x() * thirdDerivative.y() - thirdDerivative.x() * derivative.y())) / (2d * Math.pow(derivative.norm(), 5d));
    }

    @Override
    public double getMeanCurvature() {
        return IntStream.range(0, getParameterSteps()).mapToDouble(i ->
                (getCurvature((double)(i) / getParameterSteps()) + getCurvature((double)(i + 1) / getParameterSteps())) / (2d * getParameterSteps())).sum();
    }

    @Override
    public double getMeanDCurvature() {
        return IntStream.range(0, getParameterSteps()).mapToDouble(i ->
                (getDCurvature((double)(i) / getParameterSteps()) + getDCurvature((double)(i + 1) / getParameterSteps())) / (2d * getParameterSteps())).sum();
    }

    public double getArcLength() {
        return IntStream.range(0, getParameterSteps()).mapToDouble(i ->
                (getDerivative((double)(i) / getParameterSteps()).norm() + getDerivative((double)(i + 1) / getParameterSteps()).norm()) / (2d * getParameterSteps())).sum();
    }

    public double getMinDistanceFromPoint(Translation2d point) {
        return IntStream.range(0, getParameterSteps()).mapToDouble(i -> evaluate((double)(i) / getParameterSteps()).distance(point)).min().getAsDouble();
    }

    public double[] getParameterAndMinDistanceFromPoint(Translation2d point) {
        double parameterValue = 0d;
        double minDistance = Double.MAX_VALUE;
        for(int i = 0; i < getParameterSteps(); i++) {
            double parameter = (double)(i) / getParameterSteps();
            double distance = evaluate(parameter).distance(point);
            if(minDistance > distance) {
                parameterValue = parameter;
                minDistance = distance;
            }
        }

        return new double[] {parameterValue, minDistance};
    }

    public int getPolynomialDegree() {
        return polynomialDegree;
    }

    public void setPolynomialDegree(int polynomialDegree) {
        this.polynomialDegree = polynomialDegree;
    }

    public double[] getCoefficients() {
        return coefficients;
    }

    public void setCoefficients(double[] coefficients) {
        this.coefficients = coefficients;
    }

    public boolean isInverted() {
        return inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public static int getParameterSteps() {
        return PARAMETER_STEPS;
    }
}
