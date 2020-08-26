package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.debugging.RobotDebug;
import com.horse.mpclib.lib.control.Obstacle;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.physics.MecanumDriveModel;
import com.horse.mpclib.lib.physics.MotorModel;
import com.horse.mpclib.lib.util.TimeProfiler;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

import java.util.ArrayList;
import java.util.List;

public abstract class Robot implements RobotDebug {
    private static final SimpleMatrix INITIAL_STATE = new SimpleMatrix(6, 1, false, new double[] {
            //(12d * 12d - 9d) * 0.0254d, 0d, 48d * 0.0254d, 0d, Math.toRadians(-180d), 0d
            9d * 0.0254d, 0d, 9d * 0.0254d, 0d, Math.toRadians(0d), 0d

            //9d * 0.0254d, 0d, 48d * 0.0254d, 0d, Math.toRadians(0d), 0d
            //9d * 0.0254d, 0d, (4d + 8d * 5d) * 0.0254d, 0d, Math.toRadians(0d), 0d
            //9d * 0.0254d, 0d, 40d * 0.0254d, 0d, Math.toRadians(-90d), 0d
    });

    private static final Pose2d INITIAL_POSE = new Pose2d(getInitialState().get(0) / 0.0254d, getInitialState().get(2) / 0.0254d, new Rotation2d(getInitialState().get(4), false));

    private static final Translation2d frontLeftWheel  = new Translation2d(-0.18d, 0.1805d);
    private static final Translation2d frontRightWheel = new Translation2d(0.1805d, 0.18d);
    private static final Translation2d backLeftWheel   = new Translation2d(-0.1805d, -0.15d);
    private static final Translation2d backRightWheel  = new Translation2d(0.18d, -0.1505d);
    private boolean stopTimer = false;

    private TimeProfiler timeProfiler;
    private double dt;

    private static SimpleMatrix state;
    private static SimpleMatrix input;
    private static SimpleMatrix wheelPositions;

    private static MecanumDriveModel driveModel;

    private static List<Obstacle> obstacles = new ArrayList<>();

    @Override
    public void init_debug() {
        setState(getInitialState());
        setInput(new SimpleMatrix(4, 1, false, new double[] {0d, 0d, 0d, 0d}));
        setWheelPositions(new SimpleMatrix(4, 1, false, new double[] {0d, 0d, 0d, 0d}));
        setTimeProfiler(new TimeProfiler(false));
        setDt(0d);
        setDriveModel(new MecanumDriveModel(
                0.001d, 18.4d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.NEVEREST_20, null)));
    }

    @Override
    public void start_debug() {
        getTimeProfiler().start();
        TimeUtil.startTime();
    }

    int frame = 0;

    @Override
    public void loop_debug() {
        if(!stopTimer) {
            ComputerDebugger.send(MessageOption.TIME);
        }

        setDt(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
        //setDt(1 / 240d);
        setState(getDriveModel().simulate(getState(), getInput(), getDt()));
        //setState(getDriveModel().simulateDynamics(getState(), getInput(), getDt()));

        /*if(!stopTimer) {
            setWheelPositions(getDriveModel().updateWheelAngularPositions(getWheelPositions(), getState(), getDt()));
            SimpleMatrix positions = getWheelPositions().scale(180d / Math.PI);
            if(frame % 8 == 0) {
                SimpleMatrix relativeState = getState().minus(getInitialState()).plus(new SimpleMatrix(6, 1, false, new double[] {
                        -1.4827d, 0d, -1.809d, 0d, Math.toRadians(0d), 0d
                }));

                double x = getInitialState().get(2) - getState().get(2);*/

                /*System.out.println((int)(frame / 8d) + 1 + "\t" + x + "\t" +
                        relativeState.get(0) + "\t" + (relativeState.get(4) * 180d / Math.PI) + "\t" + positions.get(0) + "\t" +
                        positions.get(1) + "\t" + positions.get(2) + "\t" + positions.get(3));*/

                /*System.out.println((int)(frame / 8d) + 1 + "\t" + relativeState.get(0) + "\t" +
                        relativeState.get(2) + "\t" + (relativeState.get(4) * 180d / Math.PI) + "\t" + positions.get(0) + "\t" +
                        positions.get(1) + "\t" + positions.get(2) + "\t" + positions.get(3));
            }

            frame++;
        }*/
    }

    @Override
    public void sendMotionProfileData() {

    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d(getState().get(0) / 0.0254d, getState().get(2) / 0.0254d, new Rotation2d(getState().get(4), false));
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public double getDt() {
        return dt;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public static SimpleMatrix getInitialState() {
        return INITIAL_STATE;
    }

    public static SimpleMatrix getState() {
        return state;
    }

    public static void setState(SimpleMatrix state) {
        Robot.state = state;
    }

    public static MecanumDriveModel getDriveModel() {
        return driveModel;
    }

    public static void setDriveModel(MecanumDriveModel driveModel) {
        Robot.driveModel = driveModel;
    }

    public static SimpleMatrix getInput() {
        return input;
    }

    public static void setInput(SimpleMatrix input) {
        Robot.input = input;
    }

    public void stopTimer() {
        this.stopTimer = true;
    }

    public static SimpleMatrix getWheelPositions() {
        return wheelPositions;
    }

    public static void setWheelPositions(SimpleMatrix wheelPositions) {
        Robot.wheelPositions = wheelPositions;
    }

    public static Pose2d getInitialPose() {
        return INITIAL_POSE;
    }

    public static List<Obstacle> getObstacles() {
        return obstacles;
    }

    public static void setObstacles(List<Obstacle> obstacles) {
        Robot.obstacles = obstacles;
    }
}
