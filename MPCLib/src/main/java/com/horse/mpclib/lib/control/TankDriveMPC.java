package com.horse.mpclib.lib.control;

import org.ejml.data.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

public class TankDriveMPC {
    private TankDriveILQR tankDriveILQR;
    private SimpleMatrix[] simulatedStates;
    private SimpleMatrix[] simulatedInputs;
    private SimpleMatrix[] A;
    private SimpleMatrix[] B;
    private SimpleMatrix[] P;
    private SimpleMatrix[] p;
    private SimpleMatrix[] K;
    private SimpleMatrix[] l;

    private SimpleMatrix initialState;
    private SimpleMatrix desiredState;

    private double currentRuntime;
    private boolean isFirstIteration;

    public TankDriveMPC(TankDriveILQR tankDriveILQR) {
        setTankDriveILQR(tankDriveILQR);
        setFirstIteration(true);
    }

    public void initialIteration(SimpleMatrix initialState, Pose2d desiredPose) {
        initialIteration(initialState, new SimpleMatrix(5, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, desiredPose.getTranslation().y() * 0.0254d, desiredPose.getRotation().getRadians(), 0d, 0d
        }));
    }

    public void initialIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        setInitialState(initialState);
        setDesiredState(desiredState);
        getTankDriveILQR().runLQR(initialState);
        K = getTankDriveILQR().getK();
        l = new SimpleMatrix[TankDriveILQR.getHorizonStep()];
        for(int i = 0; i < l.length; i++) {
            l[i] = new SimpleMatrix(2, 1, true, new double[] {
                    0d, 0d
            });
        }
    }

    public void simulateIteration() {
        simulateIteration(getInitialState(), getDesiredState());
    }

    public void simulateIteration(Pose2d desiredPose) {
        simulateIteration(getInitialState(), desiredPose);
    }

    public void simulateIteration(SimpleMatrix desiredState) {
        simulateIteration(getInitialState(), desiredState);
    }

    public void simulateIteration(SimpleMatrix initialState, Pose2d desiredPose) {
        simulateIteration(initialState, new SimpleMatrix(5, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, desiredPose.getTranslation().y() * 0.0254d, desiredPose.getRotation().getRadians(), 0d, 0d
        }));
    }

    public void simulateIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        if(getSimulatedStates() == null) {
            setSimulatedStates(new SimpleMatrix[TankDriveILQR.getHorizonStep() + 1]);
            setSimulatedInputs(new SimpleMatrix[TankDriveILQR.getHorizonStep()]);
            getSimulatedStates()[0] = initialState;
            setA(new SimpleMatrix[TankDriveILQR.getHorizonStep()]);
            setB(new SimpleMatrix[TankDriveILQR.getHorizonStep()]);
            for(int i = 1; i <= TankDriveILQR.getHorizonStep(); i++) {
                getA()[i - 1] = getTankDriveILQR().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], TankDriveILQR.getDt());
                getB()[i - 1] = getTankDriveILQR().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], TankDriveILQR.getDt());
                getSimulatedInputs()[i - 1] = getTankDriveILQR().getOptimalInput(i - 1, getSimulatedStates()[i - 1], desiredState);
                //getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
                getSimulatedStates()[i] = getTankDriveILQR().getModel().simulateNonlinear(getSimulatedStates()[i - 1], getSimulatedInputs()[i - 1], TankDriveILQR.getDt());
            }
        } else {
            for(int i = 1; i <= TankDriveILQR.getHorizonStep(); i++) {
                getA()[i - 1] = getTankDriveILQR().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], TankDriveILQR.getDt());
                getB()[i - 1] = getTankDriveILQR().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], TankDriveILQR.getDt());
                getSimulatedInputs()[i - 1] = getOptimalInput(i - 1, getSimulatedStates()[i - 1], 0.001d);
                //getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
                getSimulatedStates()[i] = getTankDriveILQR().getModel().simulateNonlinear(getSimulatedStates()[i - 1], getSimulatedInputs()[i - 1], TankDriveILQR.getDt());
            }
        }
    }

    public void runSLQ() {
        P = new SimpleMatrix[TankDriveILQR.getHorizonStep()];
        p = new SimpleMatrix[TankDriveILQR.getHorizonStep()];
        P[P.length - 1] = TankDriveILQR.getTerminationCost();
        l[l.length - 1] = getLinearStateCost(TankDriveILQR.getHorizonStep(), TankDriveILQR.getTerminationCost());
        setCurrentRuntime(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
        solveRiccatiEquations(TankDriveILQR.getHorizonStep() - 1);
        setFirstIteration(false);
    }

    public void solveRiccatiEquations(int timeStep) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix A = getA()[timeStep];
        SimpleMatrix B = getB()[timeStep];

        SimpleMatrix Q = TankDriveILQR.getStateCost(timeStep);
        SimpleMatrix R = TankDriveILQR.getInputCost();
        SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).pseudoInverse();
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
        K[timeStep - 1] = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();

        SimpleMatrix S = P[timeStep].minus(P[timeStep].mult(B).mult(inverse.invert()).mult(B.transpose()).mult(P[timeStep]));
        l[timeStep - 1] = A.transpose().mult(S).mult(P[timeStep].invert()).mult(l[timeStep]);

        solveRiccatiEquations(--timeStep);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, double alpha) {
        if(getSimulatedInputs() != null && P != null && timeStep < getSimulatedInputs().length - 1) {
            SimpleMatrix A = getTankDriveILQR().getModel().stateTransitionMatrix(state, TankDriveILQR.getDt());
            SimpleMatrix B = getTankDriveILQR().getModel().inputTransitionMatrix(state, TankDriveILQR.getDt());
            SimpleMatrix K;
            try {
                SimpleMatrix inverse = TankDriveILQR.getInputCost().plus(B.transpose().mult(P[timeStep].mult(B))).invert();
                K = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
                return TankDriveILQR.limitInput(getSimulatedInputs()[timeStep].plus(K.mult(state.minus(getSimulatedStates()[timeStep]))).minus(
                        TankDriveILQR.getInputCost().plus(getB()[timeStep].transpose().mult(P[timeStep])
                                .mult(B)).invert().mult(B.transpose()).mult(l[timeStep]).scale(1 / 2d)));
            } catch(SingularMatrixException e) {
                P[timeStep].print();
                l[timeStep].print();
            }
        } else if(timeStep < getTankDriveILQR().getK().length - 1) {
            return getTankDriveILQR().getOptimalInput(timeStep, state, getDesiredState());
        }

        return new SimpleMatrix(2, 1, true, new double[] {
                0d, 0d
        });
    }

    public SimpleMatrix getLinearStateCost(SimpleMatrix state, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(state).scale(-1 / 2d);
    }

    public SimpleMatrix getLinearStateCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedStates()[timeStep]).scale(-1 / 2d);
    }

    public SimpleMatrix getLinearInputCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedInputs()[timeStep]).scale(-1 / 2d);
    }

    public TankDriveILQR getTankDriveILQR() {
        return tankDriveILQR;
    }

    public void setTankDriveILQR(TankDriveILQR tankDriveILQR) {
        this.tankDriveILQR = tankDriveILQR;
    }

    public SimpleMatrix[] getSimulatedStates() {
        return simulatedStates;
    }

    public void setSimulatedStates(SimpleMatrix[] simulatedStates) {
        this.simulatedStates = simulatedStates;
    }

    public SimpleMatrix[] getA() {
        return A;
    }

    public void setA(SimpleMatrix[] a) {
        A = a;
    }

    public SimpleMatrix[] getB() {
        return B;
    }

    public void setB(SimpleMatrix[] b) {
        B = b;
    }

    public SimpleMatrix[] getSimulatedInputs() {
        return simulatedInputs;
    }

    public void setSimulatedInputs(SimpleMatrix[] simulatedInputs) {
        this.simulatedInputs = simulatedInputs;
    }

    public SimpleMatrix getInitialState() {
        return initialState;
    }

    public void setInitialState(SimpleMatrix initialState) {
        this.initialState = initialState;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }

    public double getCurrentRuntime() {
        return currentRuntime;
    }

    public void setCurrentRuntime(double currentRuntime) {
        this.currentRuntime = currentRuntime;
    }

    public boolean isFirstIteration() {
        return isFirstIteration;
    }

    public void setFirstIteration(boolean firstIteration) {
        isFirstIteration = firstIteration;
    }
}
