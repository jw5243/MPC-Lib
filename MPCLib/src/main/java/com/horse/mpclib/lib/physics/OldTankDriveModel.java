package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.drivers.Motor;

/**
 * The state of the tank drive model contains 10 values:
 *     0) x-position
 *     1) x-velocity
 *     2) y-position
 *     3) y-velocity
 *     4) heading
 *     5) angular velocity
 *     6) right wheel angular velocity
 *     7) left wheel angular velocity
 */
public class OldTankDriveModel {
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

    private final double A;
    private final double B;

    public OldTankDriveModel(double robotMass, double wheelInertia, double robotInertia,
                             double wheelRadius, double robotWidth, MotorModel motorModel) {
        this(robotMass, wheelInertia, robotInertia, wheelRadius, motorModel.getGearRatio(), robotWidth,
                motorModel.getNominalVoltage(), motorModel.getStallTorque(), motorModel.getStallCurrent(),
                motorModel.getFreeSpeed(), motorModel.getFreeCurrent(), motorModel.getEfficiency());
    }

    public OldTankDriveModel(double robotMass, double wheelInertia, double robotInertia,
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

        A = this.robotMass * this.wheelRadius * this.wheelRadius / 4d +
                this.robotInertia * this.wheelRadius * this.wheelRadius / (this.robotWidth * this.robotWidth) + this.wheelInertia;
        B = this.robotMass * this.wheelRadius * this.wheelRadius / 4d -
                this.robotInertia * this.wheelRadius * this.wheelRadius / (this.robotWidth * this.robotWidth);
    }

    public static void main(String... args) {
        OldTankDriveModel model = new OldTankDriveModel(15.75d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2, 0.5613d,
                0.1d / 2, 16d * 0.0254, MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null));

        SimpleMatrix state = new SimpleMatrix(8, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d
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
        double cosTheta = Math.cos(state.get(4));
        double sinTheta = Math.sin(state.get(4));
        double thetaDot = state.get(5);
        double leftWheelVelocity = state.get(7);
        double rightWheelVelocity = state.get(6);

        double M14 = -(cosTheta*(leftWheelVelocity + rightWheelVelocity)*thetaDot*wheelRadius)/2;
        double M15 = -((leftWheelVelocity + rightWheelVelocity)*sinTheta*wheelRadius)/2;
        double M16 = (wheelRadius*(-(sinTheta*thetaDot) - (cosTheta*efficiency*gearRatio*gearRatio*kT*kV)/(resistance*wheelInertia)))/2;
        double M17 = M16;
        double M34 = -((leftWheelVelocity + rightWheelVelocity)*sinTheta*thetaDot*wheelRadius)/ 2;
        double M35 = (cosTheta*(leftWheelVelocity + rightWheelVelocity)*wheelRadius)/2;
        double M36 = ((cosTheta*thetaDot - (efficiency*gearRatio*gearRatio*kT*kV*sinTheta)/(resistance*wheelInertia))*wheelRadius)/2;
        double M37 = M36;
        double M56 = -(efficiency * gearRatio * gearRatio * kT * kV * wheelRadius)/(resistance * robotWidth * wheelInertia);
        double M57 = -M56;
        double M66 = -((efficiency*gearRatio*gearRatio*kT*kV)/(resistance*wheelInertia));
        double M77 = M66;

        return new SimpleMatrix(8, 8, true, new double[] {
                0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, M14, M15, M16, M17,
                0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, M34, M35, M36, M37,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, M56, M57,
                0, 0, 0, 0, 0, 0, M66, 0,
                0, 0, 0, 0, 0, 0, 0, M77
        }).scale(dt).plus(SimpleMatrix.identity(8));
    }

    public SimpleMatrix inputTransitionMatrix(SimpleMatrix state, double dt) {
        double cosTheta = Math.cos(state.get(4));
        double sinTheta = Math.sin(state.get(4));

        double M10 = (cosTheta*efficiency*gearRatio*kT*nominalVoltage*wheelRadius)/ (2*resistance*wheelInertia);
        double M11 = M10;
        double M30 = (efficiency*gearRatio*kT*nominalVoltage*sinTheta*wheelRadius)/ (2*resistance*wheelInertia);
        double M31 = M30;
        double M50 = (efficiency*gearRatio*kT*nominalVoltage*wheelRadius)/(resistance*robotWidth* wheelInertia);
        double M51 = -M50;
        double M60 = (efficiency*gearRatio*kT*nominalVoltage)/(resistance*wheelInertia);
        double M71 = M60;

        return new SimpleMatrix(8, 2, true, new double[] {
                0, 0,
                M10, M11,
                0, 0,
                M30, M31,
                0, 0,
                M50, M51,
                M60, 0,
                0, M71
        }).scale(dt);
    }

    public SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt) {
        return stateTransitionMatrix(state, dt).mult(state).plus(inputTransitionMatrix(state, dt).mult(input));
    }

    public SimpleMatrix simulateNonlinear(SimpleMatrix state, SimpleMatrix input, double dt) {
        double cosTheta = Math.cos(state.get(4));
        double sinTheta = Math.sin(state.get(4));

        //final double v = (A * A - B * B) * wheelRadius;

        //double rightWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * (-B * input.get(1) + A * input.get(0)) + gearRatio * kV * (B * state.get(7) - A * state.get(6))) / v;
        //double leftWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * (A * input.get(1) - B * input.get(0)) + gearRatio * kV * (-A * state.get(7) + B * state.get(6))) / v;

        double rightWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * input.get(0) - kV * gearRatio * state.get(6)) / (resistance * wheelRadius);
        double leftWheelAcceleration = gearRatio * efficiency * kT * (nominalVoltage * input.get(1) - kV * gearRatio * state.get(7)) / (resistance * wheelRadius);
        double accelerationX = -wheelRadius * sinTheta * state.get(5) * (state.get(6) + state.get(7)) / 2d + wheelRadius * cosTheta * (leftWheelAcceleration + rightWheelAcceleration) / 2d;
        double accelerationY = wheelRadius * cosTheta * state.get(5) * (state.get(6) + state.get(7)) / 2d + wheelRadius * sinTheta * (leftWheelAcceleration + rightWheelAcceleration) / 2d;
        double angularAcceleration = wheelRadius * (-leftWheelAcceleration + rightWheelAcceleration) / robotWidth;

        return state.plus(new SimpleMatrix(8, 1, true, new double[] {
                state.get(1),
                accelerationX,
                state.get(3),
                accelerationY,
                state.get(5),
                angularAcceleration,
                rightWheelAcceleration,
                leftWheelAcceleration
        }).scale(dt));
    }

    public SimpleMatrix simulateRungeKutta(SimpleMatrix state, SimpleMatrix input, double dt) {
        return state;
    }
}
