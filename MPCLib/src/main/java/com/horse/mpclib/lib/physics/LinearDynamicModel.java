package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;

public interface LinearDynamicModel extends DynamicModel {
    SimpleMatrix stateTransitionMatrix(double dt);
    SimpleMatrix inputTransitionMatrix(double dt);

    default SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt) {
        return stateTransitionMatrix(dt).mult(state).plus(inputTransitionMatrix(dt).mult(input));
    }
}
