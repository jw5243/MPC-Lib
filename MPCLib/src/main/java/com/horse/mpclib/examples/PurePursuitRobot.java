package com.horse.mpclib.examples;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Pose2d;
import com.horse.mpclib.lib.geometry.Rotation2d;
import com.horse.mpclib.lib.geometry.Translation2d;

public class PurePursuitRobot extends Robot {
    private Pose2d finalPose = new Pose2d(120d, 120d, new Rotation2d(Math.toRadians(90d), false));
    private Pose2d robotPower;

    @Override
    public void init_debug() {
        super.init_debug();
        robotPower = new Pose2d();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        Translation2d displacement = finalPose.getTranslation().translateBy(getFieldPosition().getTranslation().inverse());
        double turnPower = (finalPose.getRotation().getRadians() - getFieldPosition().getRotation().getRadians()) / Math.toRadians(90d);
        robotPower = new Pose2d(displacement.scale(1d / 25d).rotateBy(getFieldPosition().getRotation().inverse()), new Rotation2d(turnPower, false));
        if(displacement.norm() < 0.1d) {
            robotPower = new Pose2d();
            stopTimer();
        }

        try {
            if(displacement.norm() != 0) {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(
                        new Line2d(getFieldPosition().getTranslation(), getFieldPosition().getTranslation().translateBy(
                                displacement.scale(20d / displacement.norm())
                        ))
                ));
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        setInput(convertPowerToInput(robotPower));
    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d(getState().get(0) / 0.0254d, getState().get(2) / 0.0254d, new Rotation2d(getState().get(4), false));
    }

    public static SimpleMatrix convertPowerToInput(Pose2d power) {
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        double frontLeft  = x - y - turn;
        double frontRight = x + y + turn;
        double backLeft   = x + y - turn;
        double backRight  = x - y + turn;

        double highestPower = Math.abs(frontLeft);
        if(highestPower < Math.abs(frontRight)) {
            highestPower = Math.abs(frontRight);
        }

        if(highestPower < Math.abs(backLeft)) {
            highestPower = Math.abs(backLeft);
        }

        if(highestPower < Math.abs(backRight)) {
            highestPower = Math.abs(backRight);
        }

        double normalization = highestPower > 1d ? 1d / highestPower : 1d;

        return new SimpleMatrix(4, 1, false, new double[] {
                frontLeft * normalization,
                frontRight * normalization,
                backLeft * normalization,
                backRight * normalization
        });
    }
}
