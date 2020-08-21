package com.horse.mpclib.lib.control;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.examples.TankDriveRobot;

public class TankRunnableMPC implements Runnable {
    private static final int MAX_ITERATIONS = 25;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private TankDriveMPC slqDrivetrain;
    private double policyLag;

    private SimpleMatrix desiredState;

    public TankRunnableMPC() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public TankDriveMPC slq() {
        return slq(getDesiredState());
    }

    public TankDriveMPC slq(SimpleMatrix desiredState) {
        TankDriveMPC slq = new TankDriveMPC(new TankDriveILQR(TankDriveRobot.getTankDriveModel()));
        if(getDesiredState() == null) {
            setDesiredState(TankDriveRobot.getTankState());
        }

        slq.initialIteration(TankDriveRobot.getTankState(), desiredState);
        for(int i = 0; i < getMaxIterations(); i++) {
            slq.simulateIteration();
            slq.runSLQ();
        }

        slq.simulateIteration();
        return slq;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            if(!isReadyToUpdate()) {
                getPolicyTimeProfiler().start();
                setSlqDrivetrain(slq(getDesiredState()));
                getTimeProfiler().update(true);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                setReadyToUpdate(true);
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void updateSLQ() {
        if(isReadyToUpdate() && getSlqDrivetrain() != null) {
            TankDriveRobot.setTankDriveMPC(getSlqDrivetrain());
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

    public TankDriveMPC getSlqDrivetrain() {
        return slqDrivetrain;
    }

    public void setSlqDrivetrain(TankDriveMPC slqDrivetrain) {
        this.slqDrivetrain = slqDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }

    public static int getMaxIterations() {
        return MAX_ITERATIONS;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }

    public void setDesiredState(Pose2d desiredPose) {
        this.desiredState = new SimpleMatrix(5, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, desiredPose.getTranslation().y() * 0.0254d, desiredPose.getRotation().getRadians(), 0d, 0d
        });
    }
}
