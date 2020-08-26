package com.horse.mpclib.examples;

import com.horse.mpclib.lib.control.MPCSolver;
import com.horse.mpclib.lib.control.RunnableMPC;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class RobotMPCBetter extends Robot {
    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;

    private List<SimpleMatrix> desiredStates;

    {
        setDesiredStates(new ArrayList<>());
        getDesiredStates().add(new SimpleMatrix(6, 1, false, new double[] {
                100d * 0.0254d, 0d, 100d * 0.0254d, 0d, Math.toRadians(90), 0d
        }));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setMpcSolver(new MPCSolver(1000, 0.002d, SimpleMatrix.diag(100d, 10, 100d, 10, 100d, 10),
                SimpleMatrix.diag(100d, 10, 100d, 10, 100d, 10), SimpleMatrix.diag(1d, 1d, 1d, 1d), getDriveModel()));
        try {
            getMpcSolver().initializeAndIterate(5, getInitialState(), getDesiredStates().get(0));
        } catch(InvalidDynamicModelException e) {
            e.printStackTrace();
        }

        setRunnableMPC(new RunnableMPC(5, getMpcSolver().getLqrSolver(), RobotMPCBetter::getState));
        getRunnableMPC().setDesiredState(getDesiredStates().get(0));
        new Thread(getRunnableMPC()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        MPCSolver updatedController = getRunnableMPC().getUpdatedMPC();
        if(updatedController != null) {
            setMpcSolver(updatedController);
        }

        try {
            setInput(getMpcSolver().getOptimalInput(getRunnableMPC().controllerElapsedTime(), getState()));
        } catch(InvalidDynamicModelException e) {
            e.printStackTrace();
        }
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

    public List<SimpleMatrix> getDesiredStates() {
        return desiredStates;
    }

    public void setDesiredStates(List<SimpleMatrix> desiredStates) {
        this.desiredStates = desiredStates;
    }
}
