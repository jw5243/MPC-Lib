package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.physics.DynamicModel;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.physics.LinearDynamicModel;
import com.horse.mpclib.lib.physics.NonlinearDynamicModel;

import org.ejml.simple.SimpleMatrix;

import java.util.stream.IntStream;

/**
 * This {@code class} acts as an LQR solver for controlling both linear and non-linear models. LQR is
 * a control system method by which an optimal feedback controller is generated. 'Optimal' in this sense
 * is with respect to a quadratic cost function determined by the matrices {@see terminationCost},
 * {@see intermediaryStateCost}, and {@see inputCost}. The quadratic nature ensures that the absolute minimum
 * can be determined (as long as the cost matrices are positive semi-definite of course) with exact
 * methods. This {@see class} uses a recursive, discrete (state-dependent) algebraic riccati equation (DARE)
 * in order to find this minimum. The use of the riccati equation is one of the more efficient methods
 * of quadratic optimization, with the drawback being that we must discretize the states over time,
 * hence we are not using the continuous riccati equation. This allows for easier determination of
 * the optimal control policy.
 *
 * To use this {@code class}, create a new instance of {@code LQRSolver} and to run an iteration of
 * LQR for optimization, call {@code runLQR(SimpleMatrix)} and use
 * {@code getOptimalInput(int, SimpleMatrix, SimpleMatrix)} to get the optimal control to supply to
 * your system.
 *
 * Something important to note is that LQR stands for the Linear-Quadratic Regulator, hence the
 * system must be linear in order to be optimal, not sub-optimal. Nonlinear dynamics supply a
 * sub-optimal solution to the cost function, hence iLQR is likely a better option, or even MPC.
 *
 * @see DynamicModel
 * @see #runLQR(SimpleMatrix)
 * @see #getOptimalInput(int, SimpleMatrix, SimpleMatrix)
 */
public class LQRSolver {
    private int horizonStep;
    private double dt;

    private SimpleMatrix terminationCost;
    private SimpleMatrix intermediaryStateCost;
    private SimpleMatrix inputCost;

    private DynamicModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;

    private int stateDimension;
    private int inputDimension;

    public LQRSolver(int horizonStep, double dt, SimpleMatrix terminationCost, SimpleMatrix intermediaryStateCost,
                     SimpleMatrix inputCost, DynamicModel model) {
        setHorizonStep(horizonStep);
        setDt(dt);
        setTerminationCost(terminationCost);
        setIntermediaryStateCost(intermediaryStateCost);
        setInputCost(inputCost);
        setModel(model);
        setStateDimension(getTerminationCost().numRows());
        setInputDimension(getInputCost().numRows());
    }

    public LQRSolver(LQRSolver lqrSolver) {
        this(lqrSolver.getHorizonStep(), lqrSolver.getDt(), lqrSolver.getTerminationCost(), lqrSolver.getIntermediaryStateCost(),
                lqrSolver.getInputCost(), lqrSolver.getModel());
    }

    public void runLQR(SimpleMatrix currentState) throws InvalidDynamicModelException {
        setP(new SimpleMatrix[getHorizonStep()]);
        setK(new SimpleMatrix[getHorizonStep() - 1]);
        getP()[getP().length - 1] = getStateCost(getHorizonStep());

        SimpleMatrix A = getA(currentState);
        SimpleMatrix B = getB(currentState);

        solveRiccatiEquation(getHorizonStep() - 1, A, B);
    }

    public void solveRiccatiEquation(int timeStep, SimpleMatrix A, SimpleMatrix B) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix Q = getStateCost(timeStep);
        SimpleMatrix R = getInputCost();
        try {
            SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).invert();
            getP()[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(getP()[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(getP()[timeStep].mult(A))))));
            getK()[timeStep - 1] = inverse.mult(B.transpose()).mult(getP()[timeStep]).mult(A).negative();
        } catch(Exception e) {
            System.out.println("Failed to backwards-solve riccati equation. Setting the rest of the controllers to zero.");
            while(--timeStep > 0) {
                getP()[timeStep] = new SimpleMatrix(getStateDimension(), getStateDimension());
                getK()[timeStep] = new SimpleMatrix(getInputDimension(), getStateDimension());
            }

            return;
        }

        solveRiccatiEquation(--timeStep, A, B);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix desiredState) throws InvalidDynamicModelException {
        if(timeStep < getK().length) {
            SimpleMatrix A = getA(state);
            SimpleMatrix B = getB(state);
            SimpleMatrix K;
            try {
                SimpleMatrix inverse = getInputCost().plus(B.transpose().mult(getP()[timeStep].mult(B))).invert();
                K = inverse.mult(B.transpose()).mult(getP()[timeStep]).mult(A).negative();

            } catch(Exception e) {
                K = getK()[timeStep];
            }

            return limitInput(K.mult(state.minus(desiredState)));
        }

        return new SimpleMatrix(getInputDimension(), 1);
    }

    public SimpleMatrix limitInput(SimpleMatrix control) {
        return new SimpleMatrix(control.numRows(), control.numCols(), false,
                IntStream.range(0, control.numRows()).mapToDouble(index -> control.get(index) > 1d ? 1d : control.get(index) < -1d ? -1d : control.get(index)).toArray());
    }

    public SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= getHorizonStep() - 1 ? getTerminationCost() : getIntermediaryStateCost();
    }

    public SimpleMatrix getA(SimpleMatrix currentState) throws InvalidDynamicModelException {
        SimpleMatrix A;
        if(getModel() instanceof LinearDynamicModel) {
            LinearDynamicModel linearModel = (LinearDynamicModel)(getModel());
            A = linearModel.stateTransitionMatrix(getDt());
        } else if(getModel() instanceof NonlinearDynamicModel) {
            NonlinearDynamicModel nonlinearModel = (NonlinearDynamicModel)(getModel());
            A = nonlinearModel.stateTransitionMatrix(currentState, getDt());
        } else {
            throw new InvalidDynamicModelException("Failed to incorporate dynamic model into LQRSolve.java. " +
                    "Make sure to use either the LinearDynamicMode.java or NonlinearDynamicModel.java interfaces to define your model.");
        }

        return A;
    }

    public SimpleMatrix getB(SimpleMatrix currentState) throws InvalidDynamicModelException {
        SimpleMatrix B;
        if(getModel() instanceof LinearDynamicModel) {
            LinearDynamicModel linearModel = (LinearDynamicModel)(getModel());
            B = linearModel.inputTransitionMatrix(getDt());
        } else if(getModel() instanceof NonlinearDynamicModel) {
            NonlinearDynamicModel nonlinearModel = (NonlinearDynamicModel)(getModel());
            B = nonlinearModel.inputTransitionMatrix(currentState, getDt());
        } else {
            throw new InvalidDynamicModelException("Failed to incorporate dynamic model into LQRSolve.java. " +
                    "Make sure to use either the LinearDynamicMode.java or NonlinearDynamicModel.java interfaces to define your model.");
        }

        return B;
    }

    public int getHorizonStep() {
        return horizonStep;
    }

    public void setHorizonStep(int horizonStep) {
        this.horizonStep = horizonStep;
    }

    public double getDt() {
        return dt;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public SimpleMatrix getTerminationCost() {
        return terminationCost;
    }

    public void setTerminationCost(SimpleMatrix terminationCost) {
        this.terminationCost = terminationCost;
    }

    public SimpleMatrix getIntermediaryStateCost() {
        return intermediaryStateCost;
    }

    public void setIntermediaryStateCost(SimpleMatrix intermediaryStateCost) {
        this.intermediaryStateCost = intermediaryStateCost;
    }

    public SimpleMatrix getInputCost() {
        return inputCost;
    }

    public void setInputCost(SimpleMatrix inputCost) {
        this.inputCost = inputCost;
    }

    public DynamicModel getModel() {
        return model;
    }

    public void setModel(DynamicModel model) {
        this.model = model;
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

    public int getStateDimension() {
        return stateDimension;
    }

    public void setStateDimension(int stateDimension) {
        this.stateDimension = stateDimension;
    }

    public int getInputDimension() {
        return inputDimension;
    }

    public void setInputDimension(int inputDimension) {
        this.inputDimension = inputDimension;
    }
}
