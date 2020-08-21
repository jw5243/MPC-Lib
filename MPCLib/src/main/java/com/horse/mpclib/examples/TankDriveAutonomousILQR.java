package com.horse.mpclib.examples;

import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.TankDriveILQR;
import com.horse.mpclib.lib.control.TankRunnableLQR;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.List;

public class TankDriveAutonomousILQR extends TankDriveRobot {
    private static List<Pose2d> positions = new ArrayList<>();

    {
        positions.add(new Pose2d(100d, 100d, new Rotation2d(Math.toRadians(90d), false)));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setTankDriveILQR(new TankDriveILQR(getTankDriveModel()));
        getTankDriveILQR().runLQR(getTankState(), positions.get(0));
        setTankRunnableLQR(new TankRunnableLQR());
        new Thread(getTankRunnableLQR()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getTankRunnableLQR().updateMPC();
        setTankInput(getTankDriveILQR().getOptimalInput((int)((getTankRunnableLQR().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getTankRunnableLQR().getPolicyLag()) / TankDriveILQR.getDt()), getTankState(), positions.get(0)));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2d) && positions.size() > 1) {
            positions.remove(0);
        }

        if(positions.size() == 1 && getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d)) {
            stopTimer();
        }

        try {
            for(int i = 0; i < getTankDriveILQR().getSimulatedStates().length - 1; i++) {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(
                        new Line2d(new Translation2d(
                                getTankDriveILQR().getSimulatedStates()[i].get(0) / 0.0254d,
                                getTankDriveILQR().getSimulatedStates()[i].get(1) / 0.0254d
                        ), new Translation2d(
                                getTankDriveILQR().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                getTankDriveILQR().getSimulatedStates()[i + 1].get(1) / 0.0254d
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
        TankDriveAutonomousILQR.positions = positions;
    }
}
