package com.horse.mpclib.lib.control;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.examples.Robot;

public class MecanumRunnableMPC implements Runnable {
    private static int maxIterations = 5;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private MecanumDriveMPC slqDrivetrain;
    private double policyLag;

    private SimpleMatrix desiredState;

    private static SimpleMatrix stateCost;
    private static SimpleMatrix inputCost;

    private static int horizonStep = MecanumDriveILQR.getHorizonStep();

    public MecanumRunnableMPC() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public MecanumDriveMPC mpc() {
        return mpc(getDesiredState());
    }

    public MecanumDriveMPC mpc(SimpleMatrix desiredState) {
        MecanumDriveMPC mpc = new MecanumDriveMPC(new MecanumDriveILQR(Robot.getDriveModel()));
        MecanumDriveILQR.setHorizonStep(getHorizonStep());
        if(getStateCost() != null) {
            MecanumDriveILQR.setIntermediaryStateCost(getStateCost());
        }

        if(getInputCost() != null) {
            MecanumDriveILQR.setInputCost(getInputCost());
        }

        if(getDesiredState() == null) {
            setDesiredState(Robot.getInitialState());
        }

        mpc.initialIteration(Robot.getState(), desiredState);
        for(int i = 0; i < getMaxIterations(); i++) {
            mpc.simulateIteration();
            mpc.runMPCIteration();
        }

        mpc.simulateIteration();
        return mpc;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            if(!isReadyToUpdate()) {
                getPolicyTimeProfiler().start();
                setSlqDrivetrain(mpc(getDesiredState()));
                getTimeProfiler().update(true);
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                setReadyToUpdate(true);
            }

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void updateSLQ() {
        if(isReadyToUpdate() && getSlqDrivetrain() != null) {
            Robot.setMecanumDriveMPC(getSlqDrivetrain());
            setPolicyLag(getPolicyTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
            setReadyToUpdate(false);
        }
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

    public MecanumDriveMPC getSlqDrivetrain() {
        return slqDrivetrain;
    }

    public void setSlqDrivetrain(MecanumDriveMPC slqDrivetrain) {
        this.slqDrivetrain = slqDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }

    public static void setMaxIterations(int maxIterations) {
        MecanumRunnableMPC.maxIterations = maxIterations;
    }

    public static int getMaxIterations() {
        return maxIterations;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }

    public void setDesiredState(Pose2d desiredPose) {
        this.desiredState = new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        });
    }

    public static SimpleMatrix getStateCost() {
        return stateCost;
    }

    public static void setStateCost(SimpleMatrix stateCost) {
        MecanumRunnableMPC.stateCost = stateCost;
    }

    public static SimpleMatrix getInputCost() {
        return inputCost;
    }

    public static void setInputCost(SimpleMatrix inputCost) {
        MecanumRunnableMPC.inputCost = inputCost;
    }

    public static int getHorizonStep() {
        return horizonStep;
    }

    public static void setHorizonStep(int horizonStep) {
        MecanumRunnableMPC.horizonStep = horizonStep;
    }
}
