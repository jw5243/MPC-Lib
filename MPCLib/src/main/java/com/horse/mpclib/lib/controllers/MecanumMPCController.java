package com.horse.mpclib.lib.controllers;

import com.horse.mpclib.lib.control.MPCSolver;
import com.horse.mpclib.lib.control.RunnableMPC;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.physics.MecanumDriveModel;
import com.horse.mpclib.lib.util.Util;

import org.ejml.simple.SimpleMatrix;

public class MecanumMPCController {
    private static final int DEFAULT_ITERATIONS = 5;
    private static final int DEFAULT_HORIZON_STEP = 500;
    private static final double DEFAULT_DT = 0.002d;

    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;
    private MecanumDriveModel mecanumDriveModel;

    private SimpleMatrix state;
    private SimpleMatrix input;

    private SimpleMatrix desiredState;

    public MecanumMPCController(ControllerBehavior controllerBehavior, double xImportance, double velXImportance,
                                double yImportance, double velYImportance, double headingImportance, double velHeadingImportance,
                                Pose2d initialPose) {
        setMpcSolver(new MPCSolver(getDefaultHorizonStep(), getDefaultDt(),
                SimpleMatrix.diag(xImportance, velXImportance, yImportance, velYImportance, headingImportance, velHeadingImportance).scale(controllerBehavior.getCostFactor()),
                SimpleMatrix.identity(4), null));
        setState(Util.convertPoseToState(initialPose));
        setInput(new SimpleMatrix(4, 1));
    }

    public void init() throws InvalidDynamicModelException {
        getMpcSolver().initializeAndIterate(getDefaultIterations(), getState(), getDesiredState());
        setRunnableMPC(new RunnableMPC(getDefaultIterations(), getMpcSolver(), this::getState, getDesiredState()));
        new Thread(getRunnableMPC()).start();
    }

    public void update(SimpleMatrix updatedState) throws InvalidDynamicModelException {
        setState(updatedState);
        MPCSolver updatedController = getRunnableMPC().getUpdatedMPC();
        if(updatedController != null) {
            setMpcSolver(updatedController);
        }

        setInput(getMpcSolver().getOptimalInput(getRunnableMPC().controllerElapsedTime(), getState()));
    }

    public MPCSolver getMpcSolver() {
        return mpcSolver;
    }

    public void setMpcSolver(MPCSolver mpcSolver) {
        this.mpcSolver = mpcSolver;
    }

    public RunnableMPC getRunnableMPC() {
        return runnableMPC;
    }

    public void setRunnableMPC(RunnableMPC runnableMPC) {
        this.runnableMPC = runnableMPC;
    }

    public MecanumDriveModel getMecanumDriveModel() {
        return mecanumDriveModel;
    }

    public void setMecanumDriveModel(MecanumDriveModel mecanumDriveModel) {
        this.mecanumDriveModel = mecanumDriveModel;
    }

    public SimpleMatrix getState() {
        return state;
    }

    public void setState(SimpleMatrix state) {
        this.state = state;
    }

    public SimpleMatrix getInput() {
        return input;
    }

    public void setInput(SimpleMatrix input) {
        this.input = input;
    }

    public static int getDefaultHorizonStep() {
        return DEFAULT_HORIZON_STEP;
    }

    public static double getDefaultDt() {
        return DEFAULT_DT;
    }

    public static int getDefaultIterations() {
        return DEFAULT_ITERATIONS;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }
}
