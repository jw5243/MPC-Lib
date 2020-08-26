package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.Obstacle;
import com.horse.mpclib.lib.geometry.Circle2d;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.util.Time;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

import java.util.ArrayList;
import java.util.List;

@Deprecated
public class RobotGAMPC extends Robot {
    private final int setpointCount;

    private List<Pose2d> positions = new ArrayList<>();

    private double runtime = 0d;
    private boolean isDone = false;
    private int timesHittingObstacle = 0;
    private boolean lastTimestepHittingObstacle = false;

    private Pose2d poseCheck = getInitialPose();
    private Time poseCheckTime;

    private double closestDistanceToObstacle = Double.POSITIVE_INFINITY;

    private TimeProfiler obstacleProfiler = new TimeProfiler(false);

    {
        //GF Path
        positions.add(new Pose2d(38d, 34d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 144d - 11d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 10d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 144d - 24d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(46d, 40d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(40d, 144d - 24d, new Rotation2d(Math.toRadians(-180d), false)));
        positions.add(new Pose2d(36d, 144d - 40d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 34d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 26d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 14d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false)));

        setpointCount = getPositions().size();

        ///////////////////////////////////////////////////////////////////////////////////////////

        //positions.add(new Pose2d(120, 120, new Rotation2d(Math.toRadians(90d), false)));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        getObstacles().clear();
        getObstacles().add(new Obstacle(144d - 97d, 63d, 3d, 300d));
        getObstacles().add(new Obstacle(144d - 97d, 82d, 3d, 300d));
        getObstacles().add(new Obstacle(144d - (144d - 9d), 90d, 10.5d, 300d));

        setTimesHittingObstacle(0);
        //setMecanumDriveILQR(new MecanumDriveILQR(getDriveModel()));
        //setMecanumDriveMPC(new MecanumDriveMPC(getMecanumDriveILQR()));

        //getMecanumDriveMPC().initialIteration(getState(), positions.get(0));
        //for(int i = 0; i < MecanumRunnableMPC.getMaxIterations(); i++) {
        //    getMecanumDriveMPC().simulateIteration(getState(), positions.get(0));
        //    getMecanumDriveMPC().runMPCIteration();
        //}

        //setMecanumRunnableMPC(new MecanumRunnableMPC());
        //getMecanumRunnableMPC().setDesiredState(positions.get(0));
        //new Thread(getMecanumRunnableMPC()).start();
    }

    @Override
    public void start_debug() {
        super.start_debug();
        setPoseCheckTime(TimeUtil.getCurrentRuntime());
    }

    @Override
    public void loop_debug() {
        super.loop_debug();

        //getMecanumRunnableMPC().updateSLQ();
        //setInput(getMecanumDriveMPC().getOptimalInput((int)((getMecanumRunnableMPC().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
        //        getMecanumRunnableMPC().getPolicyLag()) / MecanumDriveILQR.getDt()), getState(), 0.001d));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2.5d) && positions.size() > 1) {
            positions.remove(0);
            //getMecanumRunnableMPC().setDesiredState(positions.get(0));
        } else if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d) && positions.size() == 1) {
            stopTimer();
            setDone(true);
            setRuntime(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
            setInput(new SimpleMatrix(4, 1, true, new double[] {
                    0, 0, 0, 0
            }));
        }

        for(Obstacle obstacle : getObstacles()) {
            double distanceToObstacle = obstacle.distance(getFieldPosition().getTranslation());
            if(distanceToObstacle < getClosestDistanceToObstacle()) {
                setClosestDistanceToObstacle(distanceToObstacle);
            }

            if(obstacle.hittingObstacle(getFieldPosition().getTranslation())) {
                if(!isLastTimestepHittingObstacle()) {
                    setTimesHittingObstacle(getTimesHittingObstacle() + 1);
                    setLastTimestepHittingObstacle(true);
                    obstacleProfiler.start();
                }
            } else {
                if(obstacleProfiler.getDeltaTime(TimeUnits.SECONDS, false) > 0.5d) {
                    obstacleProfiler.getDeltaTime(true);
                    setLastTimestepHittingObstacle(false);
                }
            }
        }

        if(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) - getPoseCheckTime().getTimeValue(TimeUnits.SECONDS) > 5d) {
            setPoseCheck(getFieldPosition());
            setPoseCheckTime(TimeUtil.getCurrentRuntime());
        }

        try {
            /*for(int i = 0; i < getMecanumDriveMPC().getSimulatedStates().length - 1; i++) {
                if(!Double.isNaN(getMecanumDriveMPC().getSimulatedStates()[i].get(0)) &&
                        !Double.isNaN(getMecanumDriveMPC().getSimulatedStates()[i].get(2)) &&
                        !Double.isNaN(getMecanumDriveMPC().getSimulatedStates()[i + 1].get(0)) &&
                        !Double.isNaN(getMecanumDriveMPC().getSimulatedStates()[i + 1].get(2))) {
                    ComputerDebugger.send(MessageOption.LINE.setSendValue(
                            new Line2d(new Translation2d(
                                    getMecanumDriveMPC().getSimulatedStates()[i].get(0) / 0.0254d,
                                    getMecanumDriveMPC().getSimulatedStates()[i].get(2) / 0.0254d
                            ), new Translation2d(
                                    getMecanumDriveMPC().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                    getMecanumDriveMPC().getSimulatedStates()[i + 1].get(2) / 0.0254d
                            ))
                    ));
                }
            }*/

            for(int j = 0; j < getObstacles().size(); j++) {
                ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                        getObstacles().get(j).getLocation(), getObstacles().get(j).getObstacleRadius() / 0.0254d
                )));
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    public double getRuntime() {
        return runtime;
    }

    public void setRuntime(double runtime) {
        this.runtime = runtime;
    }

    public boolean isDone() {
        return isDone;
    }

    public void setDone(boolean done) {
        isDone = done;
    }

    public List<Pose2d> getPositions() {
        return positions;
    }

    public int getTimesHittingObstacle() {
        return timesHittingObstacle;
    }

    public void setTimesHittingObstacle(int timesHittingObstacle) {
        this.timesHittingObstacle = timesHittingObstacle;
    }

    public boolean isLastTimestepHittingObstacle() {
        return lastTimestepHittingObstacle;
    }

    public void setLastTimestepHittingObstacle(boolean lastTimestepHittingObstacle) {
        this.lastTimestepHittingObstacle = lastTimestepHittingObstacle;
    }

    public int getSetpointCount() {
        return setpointCount;
    }

    public Pose2d getPoseCheck() {
        return poseCheck;
    }

    public void setPoseCheck(Pose2d poseCheck) {
        this.poseCheck = poseCheck;
    }

    public Time getPoseCheckTime() {
        return poseCheckTime;
    }

    public void setPoseCheckTime(Time poseCheckTime) {
        this.poseCheckTime = poseCheckTime;
    }

    public double getClosestDistanceToObstacle() {
        return closestDistanceToObstacle;
    }

    public void setClosestDistanceToObstacle(double closestDistanceToObstacle) {
        this.closestDistanceToObstacle = closestDistanceToObstacle;
    }
}
