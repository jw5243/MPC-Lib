package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.drivers.Motor;

/**
 * The state of the tank drive model contains 10 values:
 *     0) x-position
 *     1) y-position
 *     2) heading
 *     3) left wheel angular velocity
 *     4) right wheel angular velocity
 */
public class TankDriveModel {
    //Mechanical constants
    private final double robotMass;
    private final double wheelInertia;
    private final double robotInertia;
    private final double wheelRadius;
    private final double gearRatio;
    private final double robotWidth;

    //Electrical constants
    private final double nominalVoltage;
    private final double stallTorque;
    private final double stallCurrent;
    private final double freeSpeed;
    private final double freeCurrent;
    private final double efficiency;

    private final double resistance;
    private final double kV;
    private final double kT;

    public TankDriveModel(double robotMass, double wheelInertia, double robotInertia,
                          double wheelRadius, double robotWidth, MotorModel motorModel) {
        this(robotMass, wheelInertia, robotInertia, wheelRadius, motorModel.getGearRatio(), robotWidth,
                motorModel.getNominalVoltage(), motorModel.getStallTorque(), motorModel.getStallCurrent(),
                motorModel.getFreeSpeed(), motorModel.getFreeCurrent(), motorModel.getEfficiency());
    }

    public TankDriveModel(double robotMass, double wheelInertia, double robotInertia,
                          double wheelRadius, double gearRatio, double robotWidth, double nominalVoltage,
                          double stallTorque, double stallCurrent, double freeSpeed, double freeCurrent,
                          double efficiency) {
        this.robotMass      = robotMass;
        this.wheelInertia   = wheelInertia;
        this.robotInertia   = robotInertia;
        this.wheelRadius    = wheelRadius;
        this.gearRatio      = gearRatio;
        this.robotWidth     = robotWidth;
        this.nominalVoltage = nominalVoltage;
        this.stallTorque    = stallTorque;
        this.stallCurrent   = stallCurrent;
        this.freeSpeed      = freeSpeed;
        this.freeCurrent    = freeCurrent;
        this.efficiency     = efficiency;

        resistance = this.nominalVoltage / this.stallCurrent;
        kV = (this.nominalVoltage - this.resistance * this.freeCurrent) / (this.freeSpeed);
        kT = this.stallTorque / this.stallCurrent;
    }

    public static void main(String... args) {
        TankDriveModel model = new TankDriveModel(15.75d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2, 0.5613d,
                0.1d / 2, 16d * 0.0254, MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null));

        SimpleMatrix state = new SimpleMatrix(5, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d
        });

        SimpleMatrix input = new SimpleMatrix(2, 1, true, new double[] {
                1d, -1d
        });

        final double dt = 0.0001;

        System.out.println(state);
        for(int i = 1; i <= 10000; i++) {
            state = model.simulate(state, input, dt);
            System.out.println(state);
        }
    }

    public SimpleMatrix stateTransitionMatrix(SimpleMatrix state, double dt) {
        double cosTheta = Math.cos(state.get(2));
        double sinTheta = Math.sin(state.get(2));

        double M03 = wheelRadius * cosTheta / 2d;
        double M04 = M03;
        double M13 = wheelRadius * sinTheta / 2d;
        double M14 = M13;
        double M23 = -wheelRadius / robotWidth;
        double M24 = -M23;
        double M33 = -gearRatio * efficiency * kT * kV * gearRatio / (resistance * wheelRadius);
        double M44 = M33;

        return new SimpleMatrix(5, 5, true, new double[] {
                0, 0, 0, M03, M04,
                0, 0, 0, M13, M14,
                0, 0, 0, M23, M24,
                0, 0, 0, M33, 0,
                0, 0, 0, 0, M44
        }).scale(dt).plus(SimpleMatrix.identity(5));
    }

    public SimpleMatrix inputTransitionMatrix(SimpleMatrix state, double dt) {
        double M30 = gearRatio * efficiency * kT * nominalVoltage / (resistance * wheelRadius);
        double M41 = M30;

        return new SimpleMatrix(5, 2, true, new double[] {
                0, 0,
                0, 0,
                0, 0,
                M30, 0,
                0, M41
        }).scale(dt);
    }

    public SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt) {
        return stateTransitionMatrix(state, dt).mult(state).plus(inputTransitionMatrix(state, dt).mult(input));
    }

    public SimpleMatrix simulateNonlinear(SimpleMatrix state, SimpleMatrix input, double dt) {
        double cosTheta = Math.cos(state.get(2));
        double sinTheta = Math.sin(state.get(2));

        double leftWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * input.get(0) - kV * gearRatio * state.get(3)) / (resistance * wheelRadius);
        double rightWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * input.get(1) - kV * gearRatio * state.get(4)) / (resistance * wheelRadius);

        return state.plus(new SimpleMatrix(5, 1, true, new double[] {
                wheelRadius * cosTheta * (state.get(3) + state.get(4)) / 2d,
                wheelRadius * sinTheta * (state.get(3) + state.get(4)) / 2d,
                wheelRadius * (state.get(4) - state.get(3)) / robotWidth,
                leftWheelAcceleration,
                rightWheelAcceleration
        }).scale(dt));
    }

    public SimpleMatrix simulateRungeKutta(SimpleMatrix state, SimpleMatrix input, double dt) {
        return state;
    }
}
