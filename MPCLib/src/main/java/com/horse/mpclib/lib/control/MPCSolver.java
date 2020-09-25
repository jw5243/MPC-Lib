package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.physics.DynamicModel;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

import org.ejml.data.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

public class MPCSolver {
    private LQRSolver lqrSolver;
    private SimpleMatrix[] simulatedStates;
    private SimpleMatrix[] simulatedInputs;
    private SimpleMatrix[] A;
    private SimpleMatrix[] B;
    private SimpleMatrix[] P;
    private SimpleMatrix[] p;
    private SimpleMatrix[] K;
    private SimpleMatrix[] l;

    private SimpleMatrix currentState;
    private SimpleMatrix desiredState;

    private List<? extends Costable> costables;

    private double currentRuntime;
    private boolean isFirstIteration;

    public MPCSolver(LQRSolver lqrSolver) {
        this(lqrSolver, new ArrayList<>());
    }

    public MPCSolver(LQRSolver lqrSolver, List<? extends Costable> costables) {
        setLqrSolver(lqrSolver);
        setFirstIteration(true);
        setCostables(costables);
    }

    public MPCSolver(int horizonStep, double dt, SimpleMatrix stateCost,
                     SimpleMatrix inputCost, DynamicModel model) {
        this(new LQRSolver(horizonStep, dt, stateCost, stateCost, inputCost, model));
    }

    public MPCSolver(int horizonStep, double dt, SimpleMatrix terminationCost, SimpleMatrix intermediaryStateCost,
                      SimpleMatrix inputCost, DynamicModel model) {
        this(new LQRSolver(horizonStep, dt, terminationCost, intermediaryStateCost, inputCost, model));
    }

    public MPCSolver(int horizonStep, double dt, SimpleMatrix stateCost,
                     SimpleMatrix inputCost, DynamicModel model, List<? extends Costable> costables) {
        this(new LQRSolver(horizonStep, dt, stateCost, stateCost, inputCost, model), costables);
    }

    public MPCSolver(int horizonStep, double dt, SimpleMatrix terminationCost, SimpleMatrix intermediaryStateCost,
                     SimpleMatrix inputCost, DynamicModel model, List<? extends Costable> costables) {
        this(new LQRSolver(horizonStep, dt, terminationCost, intermediaryStateCost, inputCost, model), costables);
    }

    public void initializeAndIterate(int iterations, SimpleMatrix currentState, SimpleMatrix desiredState) throws InvalidDynamicModelException {
        initialIteration(currentState, desiredState);
        iterate(iterations);
    }

    public void iterate(int iterations) throws InvalidDynamicModelException {
        for(int i = 0; i < iterations; i++) {
            simulateIteration();
            runMPCIteration();
        }
    }

    public void initialIteration(SimpleMatrix currentState, SimpleMatrix desiredState) throws InvalidDynamicModelException {
        setCurrentState(currentState);
        setDesiredState(desiredState);
        getLqrSolver().runLQR(getCurrentState());
        setK(getLqrSolver().getK());
        setL(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
        for(int i = 0; i < getL().length; i++) {
            getL()[i] = new SimpleMatrix(getLqrSolver().getInputDimension(), 1);
        }
    }

    public void simulateIteration() throws InvalidDynamicModelException {
        simulateIteration(getCurrentState(), getDesiredState());
    }

    public void simulateIteration(SimpleMatrix currentState, SimpleMatrix desiredState) throws InvalidDynamicModelException {
        if(getSimulatedStates() == null) {
            setSimulatedStates(new SimpleMatrix[getLqrSolver().getHorizonStep() + 1]);
            setSimulatedInputs(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
            getSimulatedStates()[0] = currentState;
            setA(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
            setB(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
            for(int i = 1; i <= getLqrSolver().getHorizonStep(); i++) {
                getA()[i - 1] = getLqrSolver().getA(getSimulatedStates()[i - 1]);
                getB()[i - 1] = getLqrSolver().getB(getSimulatedStates()[i - 1]);
                getSimulatedInputs()[i - 1] = getLqrSolver().getOptimalInput(i - 1, getSimulatedStates()[i - 1], desiredState);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        } else {
            for(int i = 1; i <= getLqrSolver().getHorizonStep(); i++) {
                getA()[i - 1] = getLqrSolver().getA(getSimulatedStates()[i - 1]);
                getB()[i - 1] = getLqrSolver().getB(getSimulatedStates()[i - 1]);
                getSimulatedInputs()[i - 1] = getOptimalInput(i - 1, getSimulatedStates()[i - 1]);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        }
    }

    public void runMPCIteration() {
        setP(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
        setp(new SimpleMatrix[getLqrSolver().getHorizonStep()]);
        getP()[getP().length - 1] = getLqrSolver().getTerminationCost();
        getL()[getL().length - 1] = getLinearStateCost(getLqrSolver().getHorizonStep(), getLqrSolver().getTerminationCost());
        setCurrentRuntime(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
        solveRiccatiEquation(getLqrSolver().getHorizonStep() - 1);
        setFirstIteration(false);
    }

    public void solveRiccatiEquation(int timeStep) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix A = getA()[timeStep];
        SimpleMatrix B = getB()[timeStep];

        SimpleMatrix Q = getLqrSolver().getStateCost(timeStep).plus(getCostablesQuadraticCost(timeStep));
        SimpleMatrix R = getLqrSolver().getInputCost();
        try {
            SimpleMatrix inverse = R.plus(B.transpose().mult(getP()[timeStep].mult(B))).invert();
            getP()[timeStep - 1] = Q.plus(A.transpose().mult(getP()[timeStep].mult(A))).minus(A.transpose().mult(getP()[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(getP()[timeStep].mult(A))))));
            getK()[timeStep - 1] = inverse.mult(B.transpose()).mult(getP()[timeStep]).mult(A).negative();

            SimpleMatrix q = getCostablesLinearCost(timeStep);

            SimpleMatrix S = getP()[timeStep].minus(getP()[timeStep].mult(B).mult(inverse.invert()).mult(B.transpose()).mult(getP()[timeStep]));
            getL()[timeStep - 1] = A.transpose().mult(S).mult(getP()[timeStep].invert()).mult(getL()[timeStep].plus(q));
        } catch(SingularMatrixException e) {
            getp()[timeStep - 1] = new SimpleMatrix(getLqrSolver().getStateDimension(), getLqrSolver().getStateDimension());
            getK()[timeStep - 1] = new SimpleMatrix(getLqrSolver().getInputDimension(), getLqrSolver().getStateDimension());
            getL()[timeStep - 1] = new SimpleMatrix(getLqrSolver().getStateDimension(), 1);
        }

        solveRiccatiEquation(--timeStep);
    }

    public SimpleMatrix getOptimalInput(double timeStamp, SimpleMatrix state) throws InvalidDynamicModelException {
        return getOptimalInput((int)(timeStamp / getLqrSolver().getDt()), state);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state) throws InvalidDynamicModelException {
        if(getSimulatedInputs() != null && getP() != null && timeStep < getSimulatedInputs().length - 1) {
            SimpleMatrix A = getLqrSolver().getA(state);
            SimpleMatrix B = getLqrSolver().getB(state);
            SimpleMatrix K;
            try {
                SimpleMatrix inverse = getLqrSolver().getInputCost().plus(B.transpose().mult(getP()[timeStep].mult(B))).invert();
                K = inverse.mult(B.transpose()).mult(getP()[timeStep]).mult(A).negative();
                return getLqrSolver().limitInput(getSimulatedInputs()[timeStep].plus(K.mult(state.minus(getSimulatedStates()[timeStep]))).minus(
                        getLqrSolver().getInputCost().plus(getB()[timeStep].transpose().mult(getP()[timeStep])
                                .mult(B)).invert().mult(B.transpose()).mult(getL()[timeStep]).scale(1 / 2d)));
            } catch(SingularMatrixException e) {

            }
        } else if(timeStep < getLqrSolver().getK().length - 1) {
            return getLqrSolver().getOptimalInput(timeStep, state, getDesiredState());
        }

        return new SimpleMatrix(getLqrSolver().getInputDimension(), 1);
    }

    public SimpleMatrix getCostablesQuadraticCost(int timeStep) {
        SimpleMatrix Q = new SimpleMatrix(getLqrSolver().getStateDimension(), getLqrSolver().getStateDimension());
        if(!isFirstIteration()) {
            try {
                for(Costable costable : getCostables()) {
                    SimpleMatrix cost = costable.getQuadraticCost(getSimulatedStates()[timeStep], timeStep, getLqrSolver().getDt());
                    if(cost != null) {
                        Q = Q.plus(cost);
                    }
                }
            } catch(NoSuchElementException e) {
            }
        }

        return Q;
    }

    public SimpleMatrix getCostablesLinearCost(int timeStep) {
        SimpleMatrix q = new SimpleMatrix(getLqrSolver().getStateDimension(), 1);
        if(!isFirstIteration()) {
            try {
                for(Costable costable : getCostables()) {
                    SimpleMatrix cost = costable.getLinearCost(getSimulatedStates()[timeStep], timeStep, getLqrSolver().getDt());
                    if(cost != null) {
                        q = q.plus(cost);
                    }
                }
            } catch(NoSuchElementException e) {
            }
        }

        return q;
    }

    public SimpleMatrix getLinearStateCost(SimpleMatrix state, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(state).scale(-1d / 2d);
    }

    public SimpleMatrix getLinearStateCost(int timeStep, SimpleMatrix cost) {
        return getLinearStateCost(getSimulatedStates()[timeStep], cost);
    }

    public LQRSolver getLqrSolver() {
        return lqrSolver;
    }

    public void setLqrSolver(LQRSolver lqrSolver) {
        this.lqrSolver = lqrSolver;
    }

    public SimpleMatrix[] getSimulatedStates() {
        return simulatedStates;
    }

    public void setSimulatedStates(SimpleMatrix[] simulatedStates) {
        this.simulatedStates = simulatedStates;
    }

    public SimpleMatrix[] getSimulatedInputs() {
        return simulatedInputs;
    }

    public void setSimulatedInputs(SimpleMatrix[] simulatedInputs) {
        this.simulatedInputs = simulatedInputs;
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

    public SimpleMatrix[] getp() {
        return p;
    }

    public void setp(SimpleMatrix[] p) {
        this.p = p;
    }

    public SimpleMatrix[] getP() {
        return P;
    }

    public void setP(SimpleMatrix[] p) {
        P = p;
    }

    public SimpleMatrix[] getK() {
        return K;
    }

    public void setK(SimpleMatrix[] k) {
        K = k;
    }

    public SimpleMatrix[] getL() {
        return l;
    }

    public void setL(SimpleMatrix[] l) {
        this.l = l;
    }

    public SimpleMatrix getCurrentState() {
        return currentState;
    }

    public void setCurrentState(SimpleMatrix currentState) {
        this.currentState = currentState;
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

    public List<? extends Costable> getCostables() {
        return costables;
    }

    public void setCostables(List<? extends Costable> costables) {
        this.costables = costables;
    }
}
