package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;

import org.ejml.simple.SimpleMatrix;

import java.util.function.Supplier;

public class RunnableLQR implements Runnable {
    private int iterations;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private LQRSolver lqrSolver;
    private double policyLag;

    private Supplier<SimpleMatrix> currentState;

    public RunnableLQR(int iterations, LQRSolver lqrSolver, Supplier<SimpleMatrix> currentState) {
        setIterations(iterations);
        setLqrSolver(lqrSolver);
        setCurrentState(currentState);
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public LQRSolver lqr() throws InvalidDynamicModelException {
        LQRSolver lqr = new LQRSolver(getLqrSolver());
        lqr.runLQR(getCurrentState().get());
        return lqr;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            try {
                if(!isReadyToUpdate()) {
                    getPolicyTimeProfiler().start();
                    setLqrSolver(lqr());
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

    public LQRSolver getUpdatedMPC() {
        if(isReadyToUpdate() && getLqrSolver() != null) {
            setPolicyLag(getPolicyTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
            setReadyToUpdate(false);
            return getLqrSolver();
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

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
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
}
