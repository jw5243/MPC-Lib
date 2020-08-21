package com.horse.mpclib.lib.control;

import org.ejml.simple.SimpleMatrix;

public interface DynamicModel {
    SimpleMatrix stateTransitionMatrix(SimpleMatrix state, double dt);
    SimpleMatrix inputTransitionMatrix(SimpleMatrix state, double dt);
    SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt);
}
