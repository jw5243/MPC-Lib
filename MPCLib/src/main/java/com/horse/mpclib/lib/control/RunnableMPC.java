package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;

import org.ejml.simple.SimpleMatrix;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

public class RunnableMPC implements Runnable {
    private int iterations;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private LQRSolver lqrSolver;
    private MPCSolver mpcSolver;
    private double policyLag;

    private SimpleMatrix desiredState;
    private List<? extends Costable> costables;

    private Supplier<SimpleMatrix> currentState;

    public RunnableMPC(int iterations, LQRSolver lqrSolver, Supplier<SimpleMatrix> currentState) {
        this(iterations, lqrSolver, currentState, new LinkedList<>());
    }

    public RunnableMPC(int iterations, LQRSolver lqrSolver, Supplier<SimpleMatrix> currentState, List<? extends Costable> costables) {
        setIterations(iterations);
        setLqrSolver(lqrSolver);
        setCurrentState(currentState);
        setCostables(costables);
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public RunnableMPC(int iterations, LQRSolver lqrSolver, Supplier<SimpleMatrix> currentState, SimpleMatrix desiredState) {
        this(iterations, lqrSolver, currentState);
        setDesiredState(desiredState);
    }

    public RunnableMPC(int iterations, LQRSolver lqrSolver, Supplier<SimpleMatrix> currentState, SimpleMatrix desiredState, List<? extends Costable> costables) {
        this(iterations, lqrSolver, currentState, costables);
        setDesiredState(desiredState);
    }

    public RunnableMPC(int iterations, MPCSolver mpcSolver, Supplier<SimpleMatrix> currentState) {
        this(iterations, mpcSolver.getLqrSolver(), currentState);
    }

    public RunnableMPC(int iterations, MPCSolver mpcSolver, Supplier<SimpleMatrix> currentState, SimpleMatrix desiredState) {
        this(iterations, mpcSolver.getLqrSolver(), currentState, desiredState);
    }

    public RunnableMPC(int iterations, MPCSolver mpcSolver, Supplier<SimpleMatrix> currentState, SimpleMatrix desiredState, List<? extends Costable> costables) {
        this(iterations, mpcSolver.getLqrSolver(), currentState, desiredState, costables);
    }

    public MPCSolver mpc(SimpleMatrix desiredState) throws InvalidDynamicModelException {
        MPCSolver mpc = new MPCSolver(new LQRSolver(getLqrSolver()), getCostables());
        mpc.initializeAndIterate(getIterations(), getCurrentState().get(), getDesiredState());
        return mpc;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            try {
                if(!isReadyToUpdate()) {
                    getPolicyTimeProfiler().start();
                    setMpcSolver(mpc(getDesiredState()));
                    getTimeProfiler().update(true);
                    Thread.sleep(1);
                    setReadyToUpdate(true);
                }

                Thread.sleep(1);
            } catch(InterruptedException | InvalidDynamicModelException e) {
                e.printStackTrace();
            }
        }
    }

    public MPCSolver getUpdatedMPC() {
        if(isReadyToUpdate() && getMpcSolver() != null) {
            setPolicyLag(getPolicyTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
            setReadyToUpdate(false);
            return getMpcSolver();
        }

        return null;
    }

    public double controllerElapsedTime() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) + getPolicyLag();
    }

    public int getIterations() {
        return iterations;
    }

    public void setIterations(int iterations) {
        this.iterations = iterations;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public TimeProfiler getPolicyTimeProfiler() {
        return policyTimeProfiler;
    }

    public void setPolicyTimeProfiler(TimeProfiler policyTimeProfiler) {
        this.policyTimeProfiler = policyTimeProfiler;
    }

    public boolean isReadyToUpdate() {
        return readyToUpdate;
    }

    public void setReadyToUpdate(boolean readyToUpdate) {
        this.readyToUpdate = readyToUpdate;
    }

    public boolean isStop() {
        return stop;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }

    public MPCSolver getMpcSolver() {
        return mpcSolver;
    }

    public void setMpcSolver(MPCSolver mpcSolver) {
        this.mpcSolver = mpcSolver;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }

    public LQRSolver getLqrSolver() {
        return lqrSolver;
    }

    public void setLqrSolver(LQRSolver lqrSolver) {
        this.lqrSolver = lqrSolver;
    }

    public Supplier<SimpleMatrix> getCurrentState() {
        return currentState;
    }

    public void setCurrentState(Supplier<SimpleMatrix> currentState) {
        this.currentState = currentState;
    }

    public List<? extends Costable> getCostables() {
        return costables;
    }

    public void setCostables(List<? extends Costable> costables) {
        this.costables = costables;
    }
}
