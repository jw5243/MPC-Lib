package com.horse.mpclib.lib.control;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.physics.MecanumDriveModel;
import com.horse.mpclib.lib.physics.MotorModel;

public class MecanumDriveILQR {
    private static int    HORIZON_STEP = 1000;//1800;
    private static final double dt           = 0.002d;

    private static SimpleMatrix TERMINATION_COST = new SimpleMatrix(6, 6, false, new double[] {
            100, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100
    });

    private static SimpleMatrix INTERMEDIARY_STATE_COST = new SimpleMatrix(6, 6, false, new double[] {
            1000, 0, 0, 0, 0, 0,
            0, 50, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 50, 0,
            0, 0, 0, 0, 0, 0,
    });

    private static SimpleMatrix INPUT_COST = new SimpleMatrix(4, 4, false, new double[] {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    });

    private MecanumDriveModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;

    public MecanumDriveILQR(MecanumDriveModel model) {
        setModel(model);
    }

    public static void main(String... args) {
        MecanumDriveModel model = new MecanumDriveModel(
                0.001d, 18.14d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null));

        MecanumDriveILQR lqr = new MecanumDriveILQR(model);
        SimpleMatrix state = new SimpleMatrix(6, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d, 0d
        });

        lqr.runLQR(state);
        System.out.println("t\tx\tvx\ty\tvy\tpsi\tvpsi");
        for(int i = 0; i < HORIZON_STEP; i++) {
            state = model.simulateDynamics(state, lqr.getOptimalInput(i, state, new Pose2d(10, 10, new Rotation2d(Math.toRadians(90d), true))), getDt());
            System.out.println((int)(1000d * i * MecanumDriveILQR.getDt()) / 1000d + "\t" + state.get(0) / 0.0254d + "\t" + state.get(1) / 0.0254d + "\t" +
                    state.get(2) / 0.0254d + "\t" + state.get(3) / 0.0254d + "\t" + state.get(4) * 180d / Math.PI + "\t" + state.get(5) * 180d / Math.PI);
        }
    }

    public void runLQR(SimpleMatrix initialState) {
        P = new SimpleMatrix[HORIZON_STEP];
        K = new SimpleMatrix[HORIZON_STEP - 1];
        P[P.length - 1] = getStateCost(HORIZON_STEP);

        SimpleMatrix A = model.stateTransitionMatrix(initialState, getDt(), true);
        SimpleMatrix B = model.inputTransitionMatrix(initialState, getDt(), false);

        solveRiccatiEquation(HORIZON_STEP - 1, A, B);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, Pose2d desiredPose) {
        return getOptimalInput(timeStep, state, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d, 0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix desiredState) {
        if(timeStep < K.length) {
            SimpleMatrix A = getModel().stateTransitionMatrix(state, getDt(), true);
            SimpleMatrix B = getModel().inputTransitionMatrix(state, getDt(), false);
            try {
                SimpleMatrix inverse = INPUT_COST.plus(B.transpose().mult(P[timeStep].mult(B))).invert();
                SimpleMatrix K = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
                return limitInput(K.mult(state.minus(desiredState)));
            } catch(Exception e) {
                return limitInput(K[timeStep].mult(state.minus(desiredState)));
            }
        }

        return new SimpleMatrix(4, 1);
    }

    public void solveRiccatiEquation(int timeStep, SimpleMatrix A, SimpleMatrix B) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix Q = getStateCost(timeStep);
        SimpleMatrix R = INPUT_COST;
        try {
            SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).invert();
            P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
            K[timeStep - 1] = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
        } catch(Exception e) {
            P[timeStep - 1] = new SimpleMatrix(6, 6, false, new double[] {
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0
            });

            K[timeStep - 1] = new SimpleMatrix(4, 6, false, new double[] {
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0
            });
        }

        solveRiccatiEquation(--timeStep, A, B);
    }

    public static SimpleMatrix limitInput(SimpleMatrix control) {
        return new SimpleMatrix(4, 1, false, new double[] {
                control.get(0) > 1d ? 1d : control.get(0) < -1d ? -1d : control.get(0),
                control.get(1) > 1d ? 1d : control.get(1) < -1d ? -1d : control.get(1),
                control.get(2) > 1d ? 1d : control.get(2) < -1d ? -1d : control.get(2),
                control.get(3) > 1d ? 1d : control.get(3) < -1d ? -1d : control.get(3)
        });
    }

    public static SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
    }

    public static double getDt() {
        return dt;
    }

    public void setModel(MecanumDriveModel model) {
        this.model = model;
    }

    public MecanumDriveModel getModel() {
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

    public static void setTerminationCost(SimpleMatrix terminationCost) {
        TERMINATION_COST = terminationCost;
    }

    public static void setIntermediaryStateCost(SimpleMatrix intermediaryStateCost) {
        INTERMEDIARY_STATE_COST = intermediaryStateCost;
    }

    public static void setInputCost(SimpleMatrix inputCost) {
        INPUT_COST = inputCost;
    }

    public static void setHorizonStep(int horizonStep) {
        HORIZON_STEP = horizonStep;
    }
}
