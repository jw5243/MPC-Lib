package com.horse.mpclib.examples;

import com.horse.mpclib.lib.control.MPCSolver;
import com.horse.mpclib.lib.control.RunnableMPC;
import com.horse.mpclib.lib.physics.InvalidDynamicModelException;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class RobotMPC extends Robot {
    private MPCSolver mpcSolver;
    private RunnableMPC runnableMPC;

    private List<SimpleMatrix> desiredStates;

    {
        setDesiredStates(new ArrayList<>());
        getDesiredStates().add(new SimpleMatrix(6, 1, false, new double[] {
                100d * 0.0254d, 0d, 100d * 0.0254d, 0d, Math.toRadians(90), 0d
        }));

        //GF Path
        /*positions.add(new Pose2d(38d, 34d, new Rotation2d(Math.toRadians(-90d), false)));
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
        positions.add(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false)));*/

        ///////////////////////////////////////////////////////////////////////////////////////////

        //positions.add(new Pose2d(120, 120, new Rotation2d(Math.toRadians(90d), false)));

        /*positions.add(new Pose2d(100d, 51d, new Rotation2d(Math.toRadians(-135d), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(120d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(100d, 26d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(106d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(102d, 42d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(106d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(102d, 34d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(106d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(102d, 18d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(106d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(102d, 12d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(106d, 116d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false)));

        obstacles.add(new Obstacle(92d, 65d, 3d, 25d));
        obstacles.add(new Obstacle(92d, 80d, 3d, 100d));
        obstacles.add(new Obstacle(144d - 9d, 90d, 9d, 200d));*/

        /*positions.add(new Pose2d(144d - 100d, 51d, new Rotation2d(Math.toRadians(135d - 180d), false)));
        positions.add(new Pose2d(144d - 104d, 120d, new Rotation2d(Math.toRadians(0d - 180d), false)));
        positions.add(new Pose2d(144d - 120d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 100d, 26d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 102d, 42d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 102d, 34d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 102d, 18d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 102d, 14d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 110d, 72d, new Rotation2d(Math.toRadians(90d - 180d), false)));*/

        /*positions.add(new Pose2d(144d - 116d, 4d + 8d * 5d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(144d - 104d, 120d, new Rotation2d(Math.toRadians(0d - 180d), false)));
        positions.add(new Pose2d(144d - 120d, 100d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 4d + 8d * 4d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(144d - 110d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 4d + 8d * 3d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(144d - 110d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 4d + 8d * 2d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(144d - 110d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 4d + 8d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(144d - 110d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 11d, new Rotation2d(Math.toRadians(-10d), false)));
        positions.add(new Pose2d(144d - 110d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 72d, new Rotation2d(Math.toRadians(90d - 180d), false)));*/

        //positions.add(new Pose2d(144d - 110d, 12d, new Rotation2d(Math.toRadians(90d - 180d), false)));

        //obstacles.add(new Obstacle(144d - 100d, 65d, 3d, 200d));
        //obstacles.add(new Obstacle(144d - 100d, 80d, 3d, 200d));

        /*obstacles.add(new Obstacle(144d - 92d, 65d, 3d, 200d));
        obstacles.add(new Obstacle(144d - 92d, 80d, 3d, 200d));
        obstacles.add(new Obstacle(144d - (144d - 9d), 90d, 10.5d, 200d));*/

        //obstacles.add(new Obstacle(88d, 45d, 3d, 0.5d));
        //obstacles.add(new Obstacle(67d, 51d, 3d, 1d));

        /*waypoints.add(new Waypoint(new SimpleMatrix(6, 6, true, new double[] {
                10000, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 10000, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        }), 1d, 1.2d, new Pose2d(
                36, 50, new Rotation2d(Math.toRadians(360d), false)
        )));*/
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

        setRunnableMPC(new RunnableMPC(5, getMpcSolver().getLqrSolver(), RobotMPC::getState));
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
