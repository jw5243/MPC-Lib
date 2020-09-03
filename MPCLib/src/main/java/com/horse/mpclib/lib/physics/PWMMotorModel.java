package com.horse.mpclib.lib.physics;

import java.util.function.DoubleUnaryOperator;

public class PWMMotorModel extends MotorModel {
    private double dutyCycle;

    public PWMMotorModel(double dutyCycle, double gearRatio, double nominalVoltage, double stallTorque, double stallCurrent,
                         double freeCurrent, double freeSpeed, double efficiency, DoubleUnaryOperator inertia,
                         DoubleUnaryOperator weightAppliedTorque, double staticFriction, double coulombFriction,
                         double viscousFriction, double stribeckPower, double stribeckVelocity) {
        super(gearRatio, nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, efficiency,
                inertia, weightAppliedTorque, staticFriction, coulombFriction, viscousFriction, stribeckPower, stribeckVelocity);
    }
}
