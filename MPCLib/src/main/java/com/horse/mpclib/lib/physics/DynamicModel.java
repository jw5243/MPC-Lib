package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;

public interface DynamicModel {
    SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt);
}
