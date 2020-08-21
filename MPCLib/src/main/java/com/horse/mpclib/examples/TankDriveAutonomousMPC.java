package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.TankDriveILQR;
import com.horse.mpclib.lib.control.TankDriveMPC;
import com.horse.mpclib.lib.control.TankRunnableMPC;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.List;

public class TankDriveAutonomousMPC extends TankDriveRobot {
    private static List<Pose2d> positions = new ArrayList<>();

    {
        positions.add(new Pose2d(144d - 100d, 51d, new Rotation2d(Math.toRadians(135d - 180d), false)));
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
        positions.add(new Pose2d(144d - 110d, 72d, new Rotation2d(Math.toRadians(90d - 180d), false)));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setTankDriveILQR(new TankDriveILQR(getTankDriveModel()));
        setTankDriveMPC(new TankDriveMPC(getTankDriveILQR()));
        getTankDriveMPC().initialIteration(getTankState(), positions.get(0));
        for(int i = 0; i < TankRunnableMPC.getMaxIterations(); i++) {
            getTankDriveMPC().simulateIteration(getTankState(), positions.get(0));
            getTankDriveMPC().runSLQ();
        }

        setTankRunnableMPC(new TankRunnableMPC());
        getTankRunnableMPC().setDesiredState(positions.get(0));
        new Thread(getTankRunnableMPC()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getTankRunnableMPC().updateSLQ();
        setTankInput(getTankDriveMPC().getOptimalInput((int)((getTankRunnableMPC().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getTankRunnableMPC().getPolicyLag()) / TankDriveILQR.getDt()), getTankState(), 0.001d));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 10d) && positions.size() > 1) {
            positions.remove(0);
            getTankRunnableMPC().setDesiredState(positions.get(0));
        } else if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d) && positions.size() == 1) {
            stopTimer();
            setInput(new SimpleMatrix(2, 1, true, new double[] {
                    0, 0
            }));
        }

        try {
            for(int i = 0; i < getTankDriveMPC().getSimulatedStates().length - 1; i++) {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(
                        new Line2d(new Translation2d(
                                getTankDriveMPC().getSimulatedStates()[i].get(0) / 0.0254d,
                                getTankDriveMPC().getSimulatedStates()[i].get(1) / 0.0254d
                        ), new Translation2d(
                                getTankDriveMPC().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                getTankDriveMPC().getSimulatedStates()[i + 1].get(1) / 0.0254d
                        ))
                ));
            }
        } catch(IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    public static List<Pose2d> getPositions() {
        return positions;
    }

    public static void setPositions(List<Pose2d> positions) {
        TankDriveAutonomousMPC.positions = positions;
    }
}
