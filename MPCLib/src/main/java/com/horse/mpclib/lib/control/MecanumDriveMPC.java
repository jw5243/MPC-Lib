package com.horse.mpclib.lib.control;

import org.ejml.data.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.physics.MecanumDriveModel;
import com.horse.mpclib.lib.physics.MotorModel;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;
import com.horse.mpclib.main.Robot;
import com.horse.mpclib.main.RobotMPC;

import java.util.ConcurrentModificationException;
import java.util.NoSuchElementException;

public class MecanumDriveMPC {
    private MecanumDriveILQR mecanumDriveILQR;
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

    public MecanumDriveMPC(MecanumDriveILQR mecanumDriveILQR) {
        setMecanumDriveILQR(mecanumDriveILQR);
        setFirstIteration(true);
    }

    public static void main(String... args) {
        MecanumDriveMPC slq = new MecanumDriveMPC(new MecanumDriveILQR(new MecanumDriveModel(
                0.001d, 18.14d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null))));

        SimpleMatrix state = new SimpleMatrix(6, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d, 0d
        });

        SimpleMatrix desiredState = new SimpleMatrix(6, 1, true, new double[] {
                10d * 0.0254d, 0d, 0d * 0.0254d, 0d, Math.toRadians(0d), 0d
        });

        int iterations = 3;
        slq.initialIteration(state, desiredState);
        for(int i = 0; i < iterations; i++) {
            slq.simulateIteration();
            slq.runMPCIteration();
        }

        System.out.println("t\tx\tvx\ty\tvy\tpsi\tvpsi");
        TimeUtil.startTime();
        for(int i = 0; i < MecanumDriveILQR.getHorizonStep() - 1; i++) {
            state = slq.getMecanumDriveILQR().getModel().stateTransitionMatrix(state, MecanumDriveILQR.getDt(), true).mult(state)
                    .plus(slq.getMecanumDriveILQR().getModel().inputTransitionMatrix(state, MecanumDriveILQR.getDt(), false).mult(
                            slq.getOptimalInput(i, state, 0.001d)));
            System.out.println((int)(1000d * i * MecanumDriveILQR.getDt()) / 1000d + "\t" + state.get(0) / 0.0254d + "\t" + state.get(1) / 0.0254d + "\t" +
                    state.get(2) / 0.0254d + "\t" + state.get(3) / 0.0254d + "\t" + state.get(4) * 180d / Math.PI + "\t" + state.get(5) * 180d / Math.PI);
        }
    }

    public void initialIteration(SimpleMatrix initialState, Pose2d desiredPose) {
        initialIteration(initialState, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public void initialIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        setInitialState(initialState);
        setDesiredState(desiredState);
        getMecanumDriveILQR().runLQR(initialState);
        K = getMecanumDriveILQR().getK();
        l = new SimpleMatrix[MecanumDriveILQR.getHorizonStep()];
        for(int i = 0; i < l.length; i++) {
            l[i] = new SimpleMatrix(4, 1, true, new double[] {
                    0d, 0d, 0d, 0d
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
        simulateIteration(initialState, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public void simulateIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        if(getSimulatedStates() == null) {
            setSimulatedStates(new SimpleMatrix[MecanumDriveILQR.getHorizonStep() + 1]);
            setSimulatedInputs(new SimpleMatrix[MecanumDriveILQR.getHorizonStep()]);
            getSimulatedStates()[0] = initialState;
            setA(new SimpleMatrix[MecanumDriveILQR.getHorizonStep()]);
            setB(new SimpleMatrix[MecanumDriveILQR.getHorizonStep()]);
            for(int i = 1; i <= MecanumDriveILQR.getHorizonStep(); i++) {
                getA()[i - 1] = getMecanumDriveILQR().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveILQR.getDt(), true);
                getB()[i - 1] = getMecanumDriveILQR().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveILQR.getDt(), false);
                getSimulatedInputs()[i - 1] = getMecanumDriveILQR().getOptimalInput(i - 1, getSimulatedStates()[i - 1], desiredState);
                //getSimulatedStates()[i] = getMecanumDriveILQR().getModel().simulate(getSimulatedStates()[i - 1], getSimulatedInputs()[i - 1], 0.002d);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        } else {
            for(int i = 1; i <= MecanumDriveILQR.getHorizonStep(); i++) {
                getA()[i - 1] = getMecanumDriveILQR().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveILQR.getDt(), true);
                getB()[i - 1] = getMecanumDriveILQR().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveILQR.getDt(), false);
                getSimulatedInputs()[i - 1] = getOptimalInput(i - 1, getSimulatedStates()[i - 1], 0.001d);
                //getSimulatedStates()[i] = getMecanumDriveILQR().getModel().simulate(getSimulatedStates()[i - 1], getSimulatedInputs()[i - 1], 0.002d);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        }
    }

    public void runMPCIteration() {
        P = new SimpleMatrix[MecanumDriveILQR.getHorizonStep()];
        p = new SimpleMatrix[MecanumDriveILQR.getHorizonStep()];
        P[P.length - 1] = MecanumDriveILQR.getTerminationCost();
        //p[p.length - 1] = getLinearStateCost(MecanumDriveILQR.getHorizonStep(), MecanumDriveILQR.getTerminationCost());
        l[l.length - 1] = getLinearStateCost(MecanumDriveILQR.getHorizonStep(), MecanumDriveILQR.getTerminationCost());
        setCurrentRuntime(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
        solveRiccatiEquations(MecanumDriveILQR.getHorizonStep() - 1);
        setFirstIteration(false);
    }

    public void solveRiccatiEquations(int timeStep) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix A = getA()[timeStep];
        SimpleMatrix B = getB()[timeStep];
        /*SimpleMatrix oldK = K[timeStep - 1];//getMecanumDriveILQR().getK()[timeStep - 1];

        SimpleMatrix obstacleQ = obstacle.getQuadraticCost(getSimulatedStates()[timeStep]);
        SimpleMatrix Q = MecanumDriveILQR.getStateCost(timeStep).plus(obstacleQ);

        SimpleMatrix R = MecanumDriveILQR.getInputCost();
        SimpleMatrix H = R.plus(B.transpose().mult(P[timeStep].mult(B)));
        SimpleMatrix G = B.transpose().mult(P[timeStep]).mult(A);
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).plus(oldK.transpose().mult(H)
                .mult(oldK)).plus(oldK.transpose().mult(G)).plus(G.transpose().mult(oldK));
        K[timeStep - 1] = H.invert().mult(G).negative();

        SimpleMatrix q = getLinearStateCost(timeStep, Q).minus(obstacleQ.mult(getSimulatedStates()[timeStep])).plus(obstacle.getLinearCost(getSimulatedStates()[timeStep]));
        SimpleMatrix r = getLinearInputCost(timeStep, R);
        SimpleMatrix g = r.plus(B.transpose().mult(p[timeStep]));

        p[timeStep - 1] = q.plus(A.transpose().mult(p[timeStep])).plus(oldK.transpose().mult(H)
                .mult(l[timeStep])).plus(G.transpose().mult(l[timeStep]).scale(2));
        l[timeStep - 1] = H.invert().mult(g).negative();*/

        SimpleMatrix Q = MecanumDriveILQR.getStateCost(timeStep).plus(getObstacleQuadraticCost(timeStep)).plus(getWaypointQuadraticCost(timeStep));
        SimpleMatrix R = MecanumDriveILQR.getInputCost();
        try {
            SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).invert();
            P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
            K[timeStep - 1] = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();

            SimpleMatrix q = /*obstacleQ.mult(getSimulatedStates()[timeStep]).plus*/(getObstacleLinearCost(timeStep)).plus(getWaypointLinearCost(timeStep));

            SimpleMatrix S = P[timeStep].minus(P[timeStep].mult(B).mult(inverse.invert()).mult(B.transpose()).mult(P[timeStep]));
            l[timeStep - 1] = A.transpose().mult(S).mult(P[timeStep].invert()).mult(l[timeStep].plus(q));
        } catch(SingularMatrixException e) {
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

            l[timeStep - 1] = new SimpleMatrix(6, 1, false, new double[] {
                    0, 0, 0, 0, 0, 0
            });
        }

        solveRiccatiEquations(--timeStep);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, double alpha) {
        if(getSimulatedInputs() != null && P != null && timeStep < getSimulatedInputs().length - 1) {
            SimpleMatrix A = getMecanumDriveILQR().getModel().stateTransitionMatrix(state, MecanumDriveILQR.getDt(), true);
            SimpleMatrix B = getMecanumDriveILQR().getModel().inputTransitionMatrix(state, MecanumDriveILQR.getDt(), false);
            SimpleMatrix K;
            try {
                SimpleMatrix inverse = MecanumDriveILQR.getInputCost().plus(B.transpose().mult(P[timeStep].mult(B))).invert();
                K = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
                return MecanumDriveILQR.limitInput(getSimulatedInputs()[timeStep].plus(K.mult(state.minus(getSimulatedStates()[timeStep]))).minus(
                        MecanumDriveILQR.getInputCost().plus(getB()[timeStep].transpose().mult(P[timeStep])
                                .mult(B)).invert().mult(B.transpose()).mult(l[timeStep]).scale(1 / 2d)));
            } catch(SingularMatrixException e) {
                //P[timeStep].print();
                //l[timeStep].print();
            }

            /*SimpleMatrix H = MecanumDriveILQR.getInputCost().plus(B.transpose().mult(P[timeStep].mult(B)));
            SimpleMatrix r = getLinearInputCost(timeStep, MecanumDriveILQR.getInputCost());
            SimpleMatrix g = r.plus(B.transpose().mult(p[timeStep]));
            SimpleMatrix l = H.invert().mult(g).negative();
            return MecanumDriveILQR.limitInput(getSimulatedInputs()[timeStep].plus(l.scale(alpha)).plus(K.mult(state.minus(getSimulatedStates()[timeStep]))));*/

            //return MecanumDriveILQR.limitInput(getSimulatedInputs()[timeStep].plus(l[timeStep].scale(alpha)).plus(K[timeStep].mult(state.minus(getSimulatedStates()[timeStep]))));
        } else if(timeStep < getMecanumDriveILQR().getK().length - 1) {
            return getMecanumDriveILQR().getOptimalInput(timeStep, state, getDesiredState());
        }

        return new SimpleMatrix(4, 1, true, new double[] {
                0d, 0d, 0d, 0d
        });
    }

    public SimpleMatrix getWaypointQuadraticCost(int timeStep) {
        SimpleMatrix Q = new SimpleMatrix(6, 6, true, new double[] {
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });

        if(!isFirstIteration()) {
            try {
                for(Waypoint waypoint : RobotMPC.getWaypoints()) {
                    Q = Q.plus(waypoint.getQuadraticCost(getCurrentRuntime() + timeStep * MecanumDriveILQR.getDt()));
                }
            } catch(NoSuchElementException e) {

            }
        }

        return Q;
    }

    public SimpleMatrix getWaypointLinearCost(int timeStep) {
        SimpleMatrix Q = new SimpleMatrix(6, 1, true, new double[] {
                0, 0, 0, 0, 0, 0
        });

        if(!isFirstIteration()) {
            try {
                for(Waypoint waypoint : RobotMPC.getWaypoints()) {
                    Q = Q.plus(getLinearStateCost(waypoint.getWaypointState(), getWaypointQuadraticCost(timeStep)));
                }
            } catch(NoSuchElementException e) {

            }
        }

        return Q;
    }

    public SimpleMatrix getObstacleQuadraticCost(int timeStep) {
        SimpleMatrix Q = new SimpleMatrix(6, 6, true, new double[] {
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });

        try {
            for(Obstacle obstacle : Robot.getObstacles()) {
                Q = Q.plus(obstacle.getQuadraticCost(getSimulatedStates()[timeStep]));
            }
        } catch(ConcurrentModificationException | NullPointerException e) {

        }

        return Q;
    }

    public SimpleMatrix getObstacleLinearCost(int timeStep) {
        SimpleMatrix Q = new SimpleMatrix(6, 1, true, new double[] {
                0, 0, 0, 0, 0, 0
        });

        try {
            for(Obstacle obstacle : Robot.getObstacles()) {
                Q = Q.plus(obstacle.getLinearCost(getSimulatedStates()[timeStep]));
            }
        } catch(ConcurrentModificationException | NullPointerException e) {

        }

        return Q;
    }

    public SimpleMatrix getLinearStateCost(SimpleMatrix state, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(state).scale(-1 / 2d);
    }

    public SimpleMatrix getLinearStateCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedStates()[timeStep]).scale(-1 / 2d);
        //return cost.transpose().mult(getSimulatedStates()[timeStep]);
    }

    public SimpleMatrix getLinearInputCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedInputs()[timeStep]).scale(-1 / 2d);
        //return cost.transpose().mult(getSimulatedInputs()[timeStep]);
    }

    public MecanumDriveILQR getMecanumDriveILQR() {
        return mecanumDriveILQR;
    }

    public void setMecanumDriveILQR(MecanumDriveILQR mecanumDriveILQR) {
        this.mecanumDriveILQR = mecanumDriveILQR;
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
