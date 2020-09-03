package com.horse.mpclib.examples;

import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.MPCSolver;
import com.horse.mpclib.lib.control.RunnableMPC;
import com.horse.mpclib.lib.geometry.Circle2d;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.Util;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class RobotMPC extends Robot {
    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;
    private TimeProfiler waitTimer = new TimeProfiler(false);
    private boolean timerStarted = false;

    private List<SimpleMatrix> desiredStates;

    {
        setDesiredStates(new ArrayList<>());

        getDesiredStates().add(Util.convertPoseToState(new Pose2d(32d, 144d - 20d - 9d - 8d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(32d, 144d - 20d - 9d - 2d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(32d, 144d - 20d - 9d - 12d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(11d, 144d - 20d - 9d - 12d, new Rotation2d(Math.toRadians(-85d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(12d, 144d - 20d - 9d - 4d, new Rotation2d(Math.toRadians(-75d), false))));

        //GF Path
        /*getDesiredStates().add(Util.convertPoseToState(new Pose2d(38d, 34d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(38d, 144d - 11d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(38d, 10d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(38d, 144d - 24d, new Rotation2d(Math.toRadians(-90d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(46d, 40d, new Rotation2d(Math.toRadians(-45d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(40d, 144d - 24d, new Rotation2d(Math.toRadians(-180d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(36d, 144d - 40d, new Rotation2d(Math.toRadians(-90), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(46d, 34d, new Rotation2d(Math.toRadians(-45d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(46d, 26d, new Rotation2d(Math.toRadians(-45d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(46d, 14d, new Rotation2d(Math.toRadians(-45d), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false))));
        getDesiredStates().add(Util.convertPoseToState(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false))));*/
    }

    @Override
    public void init_debug() {
        super.init_debug();
        //getObstacles().add(new Obstacle(144d - 92d, 65d, 3d, 20d));
        //getObstacles().add(new Obstacle(144d - 92d, 80d, 3d, 20d));
        //getObstacles().add(new Obstacle(144d - (144d - 9d), 90d, 10.5d, 20d));
        setMpcSolver(new MPCSolver(1000, 0.002d, SimpleMatrix.diag(100d, 10, 100d, 10, 100d, 10),
                SimpleMatrix.diag(100d, 25d, 100d, 25d, 10d, 1d), SimpleMatrix.diag(1d, 1d, 1d, 1d), getDriveModel(), getObstacles()));
        try {
            getMpcSolver().initializeAndIterate(5, getInitialState(), getDesiredStates().get(0));
        } catch(InvalidDynamicModelException e) {
            e.printStackTrace();
        }

        setRunnableMPC(new RunnableMPC(5, getMpcSolver(), RobotMPC::getState, getDesiredStates().get(0), getObstacles()));
        new Thread(getRunnableMPC()).start();
    }

    double lastResetTime = 0d;

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

        if(getDesiredStates().size() > 1 && Util.convertStateToPose(getDesiredStates().get(0)).distance(getFieldPosition()) < 1d
                && Math.abs(getDesiredStates().get(0).get(4) - getFieldPosition().getRotation().getRadians()) < Math.toRadians(1d)) {
            if(!timerStarted) {
                timerStarted = true;
                lastResetTime = time;
                waitTimer.reset();
            }

            if((/*waitTimer.getDeltaTime(TimeUnits.SECONDS, false) > 1d*/ time - lastResetTime > 1d && timerStarted) || getDesiredStates().size() < 3) {
                getDesiredStates().remove(0);
                getRunnableMPC().setDesiredState(getDesiredStates().get(0));
                timerStarted = false;
            }
        } else if(getDesiredStates().size() == 1 && Util.convertStateToPose(getDesiredStates().get(0)).distance(getFieldPosition()) < 0.5d
                && Math.abs(getDesiredStates().get(0).get(4) - getFieldPosition().getRotation().getRadians()) < Math.toRadians(1d)) {
            stopTimer();
            setInput(new SimpleMatrix(4, 1));
        }

        try {
            for(int i = 0; i < getMpcSolver().getSimulatedStates().length - 1; i++) {
                if(!Double.isNaN(getMpcSolver().getSimulatedStates()[i].get(0)) &&
                        !Double.isNaN(getMpcSolver().getSimulatedStates()[i].get(2)) &&
                        !Double.isNaN(getMpcSolver().getSimulatedStates()[i + 1].get(0)) &&
                        !Double.isNaN(getMpcSolver().getSimulatedStates()[i + 1].get(2))) {
                    ComputerDebugger.send(MessageOption.LINE.setSendValue(
                            new Line2d(new Translation2d(
                                    getMpcSolver().getSimulatedStates()[i].get(0) / 0.0254d,
                                    getMpcSolver().getSimulatedStates()[i].get(2) / 0.0254d
                            ), new Translation2d(
                                    getMpcSolver().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                    getMpcSolver().getSimulatedStates()[i + 1].get(2) / 0.0254d
                            ))
                    ));
                }
            }

            for(int j = 0; j < getObstacles().size(); j++) {
                ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                        getObstacles().get(j).getLocation(), getObstacles().get(j).getObstacleRadius() / 0.0254d
                )));
            }
        } catch(IllegalMessageTypeException e) {
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
