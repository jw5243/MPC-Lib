package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.main.Robot;

public class MecanumRunnableLQR implements Runnable {
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private boolean readyToUpdate;
    private boolean stop;

    private MecanumDriveILQR lqrDrivetrain;
    private double policyLag;

    public MecanumRunnableLQR() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public MecanumDriveILQR lqr() {
        MecanumDriveILQR mpc = new MecanumDriveILQR(Robot.getDriveModel());
        mpc.runLQR(Robot.getState());
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
            Robot.setMecanumDriveILQR(getLqrDrivetrain());
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

    public MecanumDriveILQR getLqrDrivetrain() {
        return lqrDrivetrain;
    }

    public void setLqrDrivetrain(MecanumDriveILQR lqrDrivetrain) {
        this.lqrDrivetrain = lqrDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }
}
