package com.horse.mpclib.lib.control;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.physics.MotorModel;
import com.horse.mpclib.lib.physics.TankDriveModel;

public class TankDriveILQR {
    private static final int    HORIZON_STEP = 1800;
    private static final double dt           = 0.005d;

    private static final SimpleMatrix TERMINATION_COST = new SimpleMatrix(5, 5, false, new double[] {
            1000, 0, 0, 0, 0,
            0, 1000, 0, 0, 0,
            0, 0, 100, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1
    });

    private static final SimpleMatrix INTERMEDIARY_STATE_COST = new SimpleMatrix(5, 5, false, new double[] {
            1000, 0, 0, 0, 0,
            0, 1000, 0, 0, 0,
            0, 0, 100, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1
    });

    private static final SimpleMatrix INPUT_COST = new SimpleMatrix(2, 2, false, new double[] {
            1, 0,
            0, 1
    });

    private TankDriveModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;

    private SimpleMatrix[] simulatedStates;

    public TankDriveILQR(TankDriveModel model) {
        setModel(model);
    }

    public static void main(String... args) {
        TankDriveModel model = new TankDriveModel(15.75d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2, 0.5613d,
                0.1d / 2, 16d * 0.0254d, MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null));

        TankDriveILQR lqr = new TankDriveILQR(model);
        SimpleMatrix state = new SimpleMatrix(5, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d
        });

        lqr.runLQR(state);
        for(int i = 0; i < HORIZON_STEP; i++) {
            state = model.simulate(state, lqr.getOptimalInput(i, state, new Pose2d(10, 0, new Rotation2d(Math.toRadians(0d), true))), getDt());
            System.out.println(state);
        }
    }

    public void runLQR(SimpleMatrix initialState) {
        P = new SimpleMatrix[HORIZON_STEP];
        K = new SimpleMatrix[HORIZON_STEP - 1];
        P[P.length - 1] = getStateCost(HORIZON_STEP);

        SimpleMatrix A = model.stateTransitionMatrix(initialState, getDt());
        SimpleMatrix B = model.inputTransitionMatrix(initialState, getDt());

        solveRiccatiEquation(HORIZON_STEP - 1, A, B);
    }

    public void runLQR(SimpleMatrix initialState, Pose2d desiredPose) {
        runLQR(initialState);
        setSimulatedStates(new SimpleMatrix[HORIZON_STEP - 1]);
        getSimulatedStates()[0] = initialState;
        for(int i = 1; i < HORIZON_STEP - 1; i++) {
            getSimulatedStates()[i] = getModel().simulateNonlinear(getSimulatedStates()[i - 1], getOptimalInput(i, getSimulatedStates()[i - 1], desiredPose), getDt());
        }
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, Pose2d desiredPose) {
        return getOptimalInput(timeStep, state, new SimpleMatrix(5, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, desiredPose.getTranslation().y() * 0.0254d, desiredPose.getRotation().getRadians(), 0d, 0d
        }));
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix desiredState) {
        if(timeStep < K.length) {
            SimpleMatrix A = getModel().stateTransitionMatrix(state, getDt());
            SimpleMatrix B = getModel().inputTransitionMatrix(state, getDt());
            SimpleMatrix inverse = INPUT_COST.plus(B.transpose().mult(P[timeStep].mult(B))).pseudoInverse();
            SimpleMatrix K = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
            return limitInput(K.mult(state.minus(desiredState)));
            //return limitInput(K[timeStep].mult(state.minus(desiredState)));
        }

        return new SimpleMatrix(2, 1);
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

    public static SimpleMatrix limitInput(SimpleMatrix control) {
        return new SimpleMatrix(2, 1, false, new double[] {
                control.get(0) > 1d ? 1d : control.get(0) < -1d ? -1d : control.get(0),
                control.get(1) > 1d ? 1d : control.get(1) < -1d ? -1d : control.get(1),
        });
    }

    public static SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
    }

    public static double getDt() {
        return dt;
    }

    public void setModel(TankDriveModel model) {
        this.model = model;
    }

    public TankDriveModel getModel() {
        return model;
    }

    public static int getHorizonStep() {
        return HORIZON_STEP;
    }

    public SimpleMatrix[] getK() {
        return K;
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

    public SimpleMatrix[] getSimulatedStates() {
        return simulatedStates;
    }

    public void setSimulatedStates(SimpleMatrix[] simulatedStates) {
        this.simulatedStates = simulatedStates;
    }
}
