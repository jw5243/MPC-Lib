package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveILQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableMPC;
import org.firstinspires.ftc.teamcode.lib.control.Obstacle;
import org.firstinspires.ftc.teamcode.lib.control.Waypoint;
import org.firstinspires.ftc.teamcode.lib.geometry.Circle2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Line2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

import java.util.ArrayList;
import java.util.List;

public class RobotMPC extends Robot {
    private static List<Pose2d> positions   = new ArrayList<>();
    private static List<Waypoint> waypoints = new ArrayList<>();

    static {
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
        getObstacles().clear();
        getObstacles().add(new Obstacle(144d - 99d, 61d, 3d, 300d));
        getObstacles().add(new Obstacle(144d - 99d, 84d, 3d, 300d));
        getObstacles().add(new Obstacle(144d - (144d - 9d), 90d, 10.5d, 300d));

        setMecanumDriveILQR(new MecanumDriveILQR(getDriveModel()));
        setMecanumDriveMPC(new MecanumDriveMPC(getMecanumDriveILQR()));

        getMecanumDriveMPC().initialIteration(getState(), positions.get(0));
        for(int i = 0; i < MecanumRunnableMPC.getMaxIterations(); i++) {
            getMecanumDriveMPC().simulateIteration(getState(), positions.get(0));
            getMecanumDriveMPC().runMPCIteration();
        }

        setMecanumRunnableMPC(new MecanumRunnableMPC());
        getMecanumRunnableMPC().setDesiredState(positions.get(0));
        new Thread(getMecanumRunnableMPC()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getMecanumRunnableMPC().updateSLQ();
        setInput(getMecanumDriveMPC().getOptimalInput((int)((getMecanumRunnableMPC().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumRunnableMPC().getPolicyLag()) / MecanumDriveILQR.getDt()), getState(), 0.001d));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2.5d) && positions.size() > 1) {
            positions.remove(0);
            getMecanumRunnableMPC().setDesiredState(positions.get(0));
        } else if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d) && positions.size() == 1) {
            stopTimer();
            setInput(new SimpleMatrix(4, 1, true, new double[] {
                    0, 0, 0, 0
            }));
        }

        try {
            for(int i = 0; i < getMecanumDriveMPC().getSimulatedStates().length - 1; i++) {
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
            }

            for(int i = 0; i < positions.size(); i++) {
                //ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(positions.get(i).getTranslation()));
            }

            for(int j = 0; j < getObstacles().size(); j++) {
                ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                        getObstacles().get(j).getLocation(), getObstacles().get(j).getObstacleRadius() / 0.0254d
                )));
            }

            for(int j = 0; j < getWaypoints().size(); j++) {
                if(getWaypoints().get(j).getDesiredTime() /*+ getWaypoints().get(j).getTemporalSpread()*/ < TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)) {
                    getWaypoints().remove(j--);
                }

                if(!getWaypoints().isEmpty() && j >= 0) {
                    ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(getWaypoints().get(j).getLocation()));
                }
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    public static List<Waypoint> getWaypoints() {
        return waypoints;
    }
}
