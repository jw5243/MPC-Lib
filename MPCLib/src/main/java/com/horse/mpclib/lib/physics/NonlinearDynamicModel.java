package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;

public interface NonlinearDynamicModel extends DynamicModel {
    SimpleMatrix stateTransitionMatrix(SimpleMatrix state, double dt);
    SimpleMatrix inputTransitionMatrix(SimpleMatrix state, double dt);

    default SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt) {
        return stateTransitionMatrix(state, dt).mult(state).plus(inputTransitionMatrix(state, dt).mult(input));
    }
}
