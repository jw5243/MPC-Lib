package com.horse.mpclib.lib.physics;

import org.ejml.simple.SimpleMatrix;

/**
 * This {@code interface} represents a dynamic model for a system in the form of a state-space model.
 * Therefore, this {@code interface} allows the user to {@code simulate} the state by specifying an
 * input and the elapsed time over which the input is being applied.
 *
 * A dynamic model can be split into two main categories: linear and non-linear. A linear system can
 * be easily described by Linear Algebra in order to control the system. A non-linear system can only
 * be approximated by a corresponding linear system to hope to be controlled via linear control theory.
 *
 * Something to note is that the best use of this {@code class} is not to implement this {@code interface}
 * directly. Rather, there exist two sub-interfaces that we make use of instead, {@code LinearDynamicModel}
 * and {@code NonlinearDynamicModel}. These sub-interfaces contain methods that supply the transition
 * matrices useful for state-space calculations. Most times the direct use of this {@code interface}
 * instead of its sub-interfaces will result in an {@code InvalidDynamicModelException}.
 *
 * @see LinearDynamicModel
 * @see NonlinearDynamicModel
 * @see InvalidDynamicModelException
 */
public interface DynamicModel {
    /**
     * This {@code method} uses the dynamic model to output the future state of the system for a
     * particular {@code input} supplied.
     *
     * @param state The current state of the system to be evolved.
     * @param input The vector of actuations for the system to use for simulation.
     * @param dt    The amount of time to simulate over using the given state and input.
     * @return      The state evolved {@code dt} in the future based on the {@code input} given.
     */
    SimpleMatrix simulate(SimpleMatrix state, SimpleMatrix input, double dt);
}
