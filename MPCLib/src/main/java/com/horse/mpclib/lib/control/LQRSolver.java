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
 * Link to MPC slides from Stanford: https://stanford.edu/class/ee364b/lectures/mpc_slides.pdf
 *
 * @see DynamicModel
 * @see #runLQR(SimpleMatrix)
 * @see #getOptimalInput(int, SimpleMatrix, SimpleMatrix)
 */
public class LQRSolver {
    /**
     * This value represents the amount of time steps to optimize in the future. This value is highly
     * dependent on {@code dt} to determine the total time over which to optimize. Quantitatively,
     * the amount of time being optimized over is the {@code horizonStep} multiplied by {@code dt}.
     *
     * Ideally, this value should be very high, but this value is strictly limited by memory and
     * computational power. Therefore, this value should be tuned so that the feedback controller
     * is as optimal as possible.
     *
     * @see #getDt()
     */
    private int horizonStep;

    /**
     * This value is the amount of time each time step takes up. Each time step should be small, as
     * in less than one second, and for very non-linear systems on the order of 0.01 to 0.001 seconds.
     * The smallness of {@code dt} can be mitigated by using a runge-kutta 4th-order approximation
     * for state simulation, rather than running a vector form of Newton's alglrotihm for state
     * evolution.
     */
    private double dt;

    /**
     * This matrix adds to the cost function a penalty at the end of the time horizon the error from
     * the desired state and the final state using the to-be-determined optimal feedback controller.
     * While this value in many cases is set equal to {@code intermediaryStateCost}, allowing for a
     * different value for the termination cost factor provides additional degrees of freedom when
     * tuning LQR.
     *
     * @see #getIntermediaryStateCost()
     */
    private SimpleMatrix terminationCost;

    /**
     * This matrix induces a penalty to the cost function for the deviation from the desired state
     * throughout the time horizon. Generally speaking, larger values (along the diagonals at least)
     * correspond to the desire for the system to reach the desired state more aggressively so as to
     * minimize the cost.
     */
    private SimpleMatrix intermediaryStateCost;

    /**
     * This matrix induces a penalty for actuating the system over the time horizon. Larger values
     * in this matrix typically correspond to less aggressive motion as more aggressive actuation
     * would lead to a higher penalty for the cost function.
     */
    private SimpleMatrix inputCost;

    /**
     * A {@code DynamicModel} represents the system which we are trying to optimize with LQR. In the
     * case of a {@code LinearDynamicModel}, LQR finds the optimal control policy with respect to the
     * given cost function. For the {@code NonlinearDynamicModel}, an sub-optimal control policy is
     * calculated, since the non-linearities are series expanded to linear order.
     */
    private DynamicModel model;

    /**
     * These matrices store valuable information regarding the calculation of the optimal control
     * policy matrices {@code K}. While it may not be completely necessary to store these matrices,
     * the state-dependence of the riccati equation speeds up the calculations for running the optimal
     * controller. Removal of storing this would require the adjustment of the parameters for some methods
     * and a slower calculation for the optimal control input after LQR has finished, if using the
     * state-dependence of the riccati solution. The advantage, to this however, is that we would
     * save a lot of memory, and thus we would be able to increase the time horizon over which we are
     * optimizing.
     *
     * @see #getK()
     */
    private SimpleMatrix[] P;

    /**
     * These matrices act as the optimal feedback controller for LQR. This is an {@code array} of
     * matrices since we are considering the general assumption that the dynamic model may be non-linear.
     * A linear controller should yield equivalent matrices for every index of {@code K}.
     */
    private SimpleMatrix[] K;

    /**
     * This value represents the amount of state variables that the state-space model contains in the
     * dynamic model. The number of dimensions can differ from the dynamic model in the case of augmenting
     * the system for non-linearities in the case of KRONIC-MPC, i.e., a variant of Koopman MPC, or
     * adding the actuation changes to the state in order to minimize the input change as well by
     * adjusting the cost function accordingly.
     */
    private int stateDimension;

    /**
     *
     */
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
        return getA(currentState, getDt());
    }

    public SimpleMatrix getA(SimpleMatrix currentState, double dt) throws InvalidDynamicModelException {
        SimpleMatrix A;
        if(getModel() instanceof LinearDynamicModel) {
            LinearDynamicModel linearModel = (LinearDynamicModel)(getModel());
            A = linearModel.stateTransitionMatrix(dt);
        } else if(getModel() instanceof NonlinearDynamicModel) {
            NonlinearDynamicModel nonlinearModel = (NonlinearDynamicModel)(getModel());
            A = nonlinearModel.stateTransitionMatrix(currentState, dt);
        } else {
            throw new InvalidDynamicModelException("Failed to incorporate dynamic model into LQRSolve.java. " +
                    "Make sure to use either the LinearDynamicMode.java or NonlinearDynamicModel.java interfaces to define your model.");
        }

        return A;
    }

    public SimpleMatrix getB(SimpleMatrix currentState) throws InvalidDynamicModelException {
        return getB(currentState, getDt());
    }

    public SimpleMatrix getB(SimpleMatrix currentState, double dt) throws InvalidDynamicModelException {
        SimpleMatrix B;
        if(getModel() instanceof LinearDynamicModel) {
            LinearDynamicModel linearModel = (LinearDynamicModel)(getModel());
            B = linearModel.inputTransitionMatrix(dt);
        } else if(getModel() instanceof NonlinearDynamicModel) {
            NonlinearDynamicModel nonlinearModel = (NonlinearDynamicModel)(getModel());
            B = nonlinearModel.inputTransitionMatrix(currentState, dt);
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
