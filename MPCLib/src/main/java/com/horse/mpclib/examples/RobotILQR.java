package com.horse.mpclib.examples;

import com.horse.mpclib.lib.control.MecanumDriveILQR;
import com.horse.mpclib.lib.control.MecanumRunnableLQR;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.List;

public class RobotILQR extends Robot {
    private List<Pose2d> positions = new ArrayList<>();

    {
        positions.add(new Pose2d(120d, 120d, new Rotation2d(Math.toRadians(180d), false)));
        /*positions.add(new Pose2d(100d, 51d, new Rotation2d(Math.toRadians(-135d), false)));
        positions.add(new Pose2d(116d, 80d, new Rotation2d(Math.toRadians(-90d), false)));
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
        positions.add(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false)));*/

        /*positions.add(new Pose2d(144d - 100d, 51d, new Rotation2d(Math.toRadians(135d - 180d), false)));
        positions.add(new Pose2d(144d - 116d, 80d, new Rotation2d(Math.toRadians(0d - 180d), false)));
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
        positions.add(new Pose2d(144d - 102d, 12d, new Rotation2d(Math.toRadians(135 - 180d), false)));
        positions.add(new Pose2d(144d - 106d, 116d, new Rotation2d(Math.toRadians(90d - 180d), false)));
        positions.add(new Pose2d(144d - 110d, 72d, new Rotation2d(Math.toRadians(90d - 180d), false)));*/
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setMecanumDriveILQR(new MecanumDriveILQR(getDriveModel()));
        getMecanumDriveILQR().runLQR(getState());
        //Arrays.stream(getMecanumDriveILQR().getK()).forEach(matrix -> matrix.scale(1 / 0.0254d).print());
        setMecanumDriveRunnableLQR(new MecanumRunnableLQR());
        new Thread(getMecanumDriveRunnableLQR()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getMecanumDriveRunnableLQR().updateMPC();
        setInput(getMecanumDriveILQR().getOptimalInput((int)((getMecanumDriveRunnableLQR().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumDriveRunnableLQR().getPolicyLag()) / MecanumDriveILQR.getDt()), getState(), positions.get(0)));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2d) && positions.size() > 1) {
            positions.remove(0);
        }

        if(positions.size() == 1 && getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d)) {
            stopTimer();
        }
    }
}
