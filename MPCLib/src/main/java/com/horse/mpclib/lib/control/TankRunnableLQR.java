package com.horse.mpclib.lib.control;

import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.examples.TankDriveAutonomousILQR;
import com.horse.mpclib.examples.TankDriveRobot;

public class TankRunnableLQR implements Runnable {
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private boolean readyToUpdate;
    private boolean stop;

    private TankDriveILQR lqrDrivetrain;
    private double policyLag;

    public TankRunnableLQR() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public TankDriveILQR lqr() {
        TankDriveILQR mpc = new TankDriveILQR(TankDriveRobot.getTankDriveModel());
        mpc.runLQR(TankDriveRobot.getTankState(), TankDriveAutonomousILQR.getPositions().get(0));
        return mpc;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            if(!isReadyToUpdate()) {
                getPolicyTimeProfiler().start();
                setLqrDrivetrain(lqr());
                getTimeProfiler().update(true);
                try {
                    Thread.sleep(10);
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

    public void updateMPC() {
        if(isReadyToUpdate() && getLqrDrivetrain() != null) {
            TankDriveRobot.setTankDriveILQR(getLqrDrivetrain());
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

    public TankDriveILQR getLqrDrivetrain() {
        return lqrDrivetrain;
    }

    public void setLqrDrivetrain(TankDriveILQR lqrDrivetrain) {
        this.lqrDrivetrain = lqrDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }
}
