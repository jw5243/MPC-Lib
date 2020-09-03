package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.physics.DynamicModel;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter {
    private LQRSolver lqrSolver;
    private SimpleMatrix C;
    private SimpleMatrix disturbanceCovariance;
    private SimpleMatrix noiseCovariance;

    public KalmanFilter(int horizonStep, double dt, SimpleMatrix C, SimpleMatrix disturbanceCovariance, SimpleMatrix noiseCovariance, DynamicModel model) {
        setC(C);
        setDisturbanceCovariance(disturbanceCovariance);
        setNoiseCovariance(noiseCovariance);
        setLqrSolver(new LQRSolver(horizonStep, dt, disturbanceCovariance, disturbanceCovariance, noiseCovariance, model) {
            @Override
            public SimpleMatrix getA(SimpleMatrix currentState) throws InvalidDynamicModelException {
                return super.getA(currentState).transpose();
            }

            @Override
            public SimpleMatrix getB(SimpleMatrix currentState) {
                return getC();
            }
        });
    }

    public void calculateKalmanGains(SimpleMatrix currentState) throws InvalidDynamicModelException {
        getLqrSolver().runLQR(currentState);
    }

    public SimpleMatrix predictState(int timeStep, SimpleMatrix lastPredictedState, SimpleMatrix measuredState,
                                     SimpleMatrix input, double dt) throws InvalidDynamicModelException {
        SimpleMatrix effectiveA = getLqrSolver().getA(lastPredictedState, dt).minus(getK(timeStep).mult(getC()));
        SimpleMatrix effectiveB = getLqrSolver().getB(lastPredictedState, dt);
        return effectiveA.mult(lastPredictedState).plus(effectiveB).mult(input).plus(getK(timeStep).mult(getC().mult(measuredState)));
    }

    public SimpleMatrix getK(int timeStep) {
        return getLqrSolver().getK()[timeStep].transpose();
    }

    public LQRSolver getLqrSolver() {
        return lqrSolver;
    }

    public void setLqrSolver(LQRSolver lqrSolver) {
        this.lqrSolver = lqrSolver;
    }

    public SimpleMatrix getC() {
        return C;
    }

    public void setC(SimpleMatrix c) {
        C = c;
    }

    public SimpleMatrix getDisturbanceCovariance() {
        return disturbanceCovariance;
    }

    public void setDisturbanceCovariance(SimpleMatrix disturbanceCovariance) {
        this.disturbanceCovariance = disturbanceCovariance;
    }

    public SimpleMatrix getNoiseCovariance() {
        return noiseCovariance;
    }

    public void setNoiseCovariance(SimpleMatrix noiseCovariance) {
        this.noiseCovariance = noiseCovariance;
    }
}
