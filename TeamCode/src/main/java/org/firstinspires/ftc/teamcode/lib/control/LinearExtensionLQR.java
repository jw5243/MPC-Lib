package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.physics.LinearExtensionModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

public class LinearExtensionLQR {
    private static final int HORIZON_STEP = 1000;
    private static final double dt        = 0.005d;

    private static final SimpleMatrix TERMINATION_COST = new SimpleMatrix(2, 2, true, new double[] {
            1000000, 0,
            0, 1000000
    });

    private static final SimpleMatrix INTERMEDIARY_STATE_COST = new SimpleMatrix(2, 2, true, new double[] {
            1000000, 0,
            0, 1000000
    });

    private static final SimpleMatrix INPUT_COST = new SimpleMatrix(1, 1, true, new double[] {
            1
    });

    //private static final SimpleMatrix INPUT_CHANGE_COST = new SimpleMatrix(1, 1, true, new double[] {
    //        1
    //});

    private LinearExtensionModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;
    private SimpleMatrix state;
    private SimpleMatrix finalState;
    private double lastInput;
    private int timeStep;

    public LinearExtensionLQR(LinearExtensionModel model, SimpleMatrix state, SimpleMatrix finalState) {
        setModel(model);
        setState(state);
        setFinalState(finalState);
    }

    public static void main(String... args) {
        final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
        final double spoolDiameter = 2d * 0.0254d; //m, 2 in
        LinearExtensionModel linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.GOBILDA_312_RPM, 2, 1d,
                        (motorPosition) -> mechanismWeight * spoolDiameter / 2d),
                spoolDiameter, 0.0025d, 0.002d
        );

        SimpleMatrix state = new SimpleMatrix(2, 1, true, new double[] {
                0d, 0d
        });

        SimpleMatrix finalState = new SimpleMatrix(2, 1, true, new double[] {
                15d * 0.0254d, 0d
        });

        LinearExtensionLQR mpc = new LinearExtensionLQR(linearExtensionModel, state, finalState);
        mpc.runLQR();

        System.out.println("t\ty\tv\tV in");
        for(int i = 0; i < HORIZON_STEP; i++) {
            double input = mpc.getOptimalInput(i, state);
            mpc.getModel().update(getDt(), input);
            state = new SimpleMatrix(2, 1, true, new double[] {
                    mpc.getModel().getPosition(),
                    mpc.getModel().getVelocity()
            });

            double timeStamp = i * dt;
            System.out.println((int)(1000d * timeStamp) / 1000d + "\t" +
                    6d * mpc.getModel().getPosition() / 0.0254d + "\t" +
                    6d * mpc.getModel().getVelocity() / 0.0254d + "\t" +
                    input);
        }
    }

    public void runLQR() {
        setTimeStep(0);
        setP(new SimpleMatrix[HORIZON_STEP]);
        setK(new SimpleMatrix[HORIZON_STEP - 1]);
        P[P.length - 1] = getStateCost(HORIZON_STEP);
        solveRiccatiEquation(HORIZON_STEP - 1, getModel().stateTransitionMatrix(getDt()), getModel().inputTransitionMatrix(getDt()));
    }

    public void solveRiccatiEquation(int timeStep, SimpleMatrix A, SimpleMatrix B) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix Q = getStateCost(timeStep);
        SimpleMatrix R = INPUT_COST;
        SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).pseudoInverse();
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
        K[timeStep - 1] = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
        solveRiccatiEquation(--timeStep, A, B);
    }

    public double getOptimalInput(int timeStep, SimpleMatrix state) {
        if(timeStep < K.length) {
            setLastInput(K[timeStep].mult(state.minus(getFinalState())).get(0));
            setLastInput(getLastInput() > 12d ? 12d : getLastInput() < -12d ? -12d : getLastInput());
        } else {
            setLastInput(0d);
        }

        return getLastInput();
    }

    public double getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix desiredState) {
        if(timeStep < K.length) {
            setLastInput(K[timeStep].mult(state.minus(desiredState)).get(0));
            setLastInput(getLastInput() > 12d ? 12d : getLastInput() < -12d ? -12d : getLastInput());
        } else {
            setLastInput(0d);
        }

        return getLastInput();
    }

    private static SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
    }

    public static int getHorizonStep() {
        return HORIZON_STEP;
    }

    public static double getDt() {
        return dt;
    }

    public static SimpleMatrix getTerminationCost() {
        return TERMINATION_COST;
    }

    public static SimpleMatrix getIntermediaryStateCost() {
        return INTERMEDIARY_STATE_COST;
    }

    public static SimpleMatrix getInputCost() {
        return INPUT_COST;
    }

    public LinearExtensionModel getModel() {
        return model;
    }

    public void setModel(LinearExtensionModel model) {
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

    public SimpleMatrix getState() {
        return state;
    }

    public void setState(SimpleMatrix state) {
        this.state = state;
    }

    public SimpleMatrix getFinalState() {
        return finalState;
    }

    public void setFinalState(SimpleMatrix finalState) {
        this.finalState = finalState;
    }

    public double getLastInput() {
        return lastInput;
    }

    public void setLastInput(double lastInput) {
        this.lastInput = lastInput;
    }

    public int getTimeStep() {
        return timeStep;
    }

    public void setTimeStep(int timeStep) {
        this.timeStep = timeStep;
    }
}
