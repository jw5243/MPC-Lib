package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class RobotPIDF extends Robot {
    private static final double MAX_POWER_ACCELERATION = 2d; //motor power / s

    private static final ControlConstants PIDF_X = new ControlConstants(1.1d / 4d, 0d, 0.07d, 0d);
    private static final ControlConstants PIDF_Y = new ControlConstants(1.1d / 4d, 0d, 0.07d, 0d);
    private static final ControlConstants PIDF_T = new ControlConstants(1d / Math.toRadians(15d), 0d, 0.1d, 0.05d);

    private static List<Pose2d> positions = new ArrayList<>();

    private static double runningSumX;
    private static double runningSumY;
    private static double runningSumT;

    private static double lastErrorX;
    private static double lastErrorY;
    private static double lastErrorT;

    private static double lastPowerFrontLeft;
    private static double lastPowerFrontRight;
    private static double lastPowerBackLeft;
    private static double lastPowerBackRight;

    static {
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
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setRunningSumX(0d);
        setRunningSumY(0d);
        setRunningSumT(0d);
        setLastErrorX(0d);
        setLastErrorY(0d);
        setLastErrorT(0d);
        setLastPowerFrontLeft(0d);
        setLastPowerFrontRight(0d);
        setLastPowerBackLeft(0d);
        setLastPowerBackRight(0d);
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2.5d) && positions.size() > 1) {
            positions.remove(0);
        } else if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d) && positions.size() == 1) {
            stopTimer();
            setInput(new SimpleMatrix(4, 1, true, new double[] {
                    0, 0, 0, 0
            }));
        }

        double dt = getDt();
        if(dt != 0 && !getPositions().isEmpty()) {
            double errorX = getPositions().get(0).getTranslation().x() - getFieldPosition().getTranslation().x();
            double errorY = getPositions().get(0).getTranslation().y() - getFieldPosition().getTranslation().y();
            double errorT = getPositions().get(0).getRotation().getRadians() - getFieldPosition().getRotation().getRadians();

            setRunningSumX(getRunningSumX() + errorX * dt);
            setRunningSumY(getRunningSumY() + errorY * dt);
            setRunningSumT(getRunningSumT() + errorT * dt);

            double outputX = getPidfX().getOutput(dt, errorX, getLastErrorX(), getRunningSumX(), 0d, 0d, true);
            double outputY = getPidfY().getOutput(dt, errorY, getLastErrorY(), getRunningSumY(), 0d, 0d, true);
            double outputT = getPidfT().getOutput(dt, errorT, getLastErrorT(), getRunningSumT(), 0d, 0d, true);
            setInput(smoothAcceleration(dt,
                    convertPowerToInput(new Pose2d(new Translation2d(outputX, outputY).rotateBy(new Rotation2d(getFieldPosition().getRotation().inverse())),
                    new Rotation2d(outputT, true)))));

            setLastErrorX(errorX);
            setLastErrorY(errorY);
            setLastErrorT(errorT);
        }

        setLastPowerFrontLeft(getInput().get(0));
        setLastPowerFrontRight(getInput().get(1));
        setLastPowerBackLeft(getInput().get(2));
        setLastPowerBackRight(getInput().get(3));
    }

    public static SimpleMatrix smoothAcceleration(double dt, SimpleMatrix input) {
        if(input.get(0) > getLastPowerFrontLeft() + getMaxPowerAcceleration() * dt) {
            input.set(0, getLastPowerFrontLeft() + getMaxPowerAcceleration() * dt);
        } else if(input.get(0) < getLastPowerFrontLeft() - getMaxPowerAcceleration() * dt) {
            input.set(0, getLastPowerFrontLeft() - getMaxPowerAcceleration() * dt);
        }

        if(input.get(1) > getLastPowerFrontRight() + getMaxPowerAcceleration() * dt) {
            input.set(1, getLastPowerFrontRight() + getMaxPowerAcceleration() * dt);
        } else if(input.get(1) < getLastPowerFrontRight() - getMaxPowerAcceleration() * dt) {
            input.set(1, getLastPowerFrontLeft() - getMaxPowerAcceleration() * dt);
        }

        if(input.get(2) > getLastPowerBackLeft() + getMaxPowerAcceleration() * dt) {
            input.set(2, getLastPowerBackLeft() + getMaxPowerAcceleration() * dt);
        } else if(input.get(2) < getLastPowerBackLeft() - getMaxPowerAcceleration() * dt) {
            input.set(2, getLastPowerBackLeft() - getMaxPowerAcceleration() * dt);
        }

        if(input.get(3) > getLastPowerBackRight() + getMaxPowerAcceleration() * dt) {
            input.set(3, getLastPowerBackRight() + getMaxPowerAcceleration() * dt);
        } else if(input.get(3) < getLastPowerBackRight() - getMaxPowerAcceleration() * dt) {
            input.set(3, getLastPowerBackRight() - getMaxPowerAcceleration() * dt);
        }

        return input;
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

    public static ControlConstants getPidfX() {
        return PIDF_X;
    }

    public static ControlConstants getPidfY() {
        return PIDF_Y;
    }

    public static ControlConstants getPidfT() {
        return PIDF_T;
    }

    public static double getRunningSumX() {
        return runningSumX;
    }

    public static void setRunningSumX(double runningSumX) {
        RobotPIDF.runningSumX = runningSumX;
    }

    public static double getRunningSumY() {
        return runningSumY;
    }

    public static void setRunningSumY(double runningSumY) {
        RobotPIDF.runningSumY = runningSumY;
    }

    public static double getRunningSumT() {
        return runningSumT;
    }

    public static void setRunningSumT(double runningSumT) {
        RobotPIDF.runningSumT = runningSumT;
    }

    public static double getLastErrorX() {
        return lastErrorX;
    }

    public static void setLastErrorX(double lastErrorX) {
        RobotPIDF.lastErrorX = lastErrorX;
    }

    public static double getLastErrorY() {
        return lastErrorY;
    }

    public static void setLastErrorY(double lastErrorY) {
        RobotPIDF.lastErrorY = lastErrorY;
    }

    public static double getLastErrorT() {
        return lastErrorT;
    }

    public static void setLastErrorT(double lastErrorT) {
        RobotPIDF.lastErrorT = lastErrorT;
    }

    public static List<Pose2d> getPositions() {
        return positions;
    }

    public static void setPositions(List<Pose2d> positions) {
        RobotPIDF.positions = positions;
    }

    public static double getMaxPowerAcceleration() {
        return MAX_POWER_ACCELERATION;
    }

    public static double getLastPowerFrontLeft() {
        return lastPowerFrontLeft;
    }

    public static void setLastPowerFrontLeft(double lastPowerFrontLeft) {
        RobotPIDF.lastPowerFrontLeft = lastPowerFrontLeft;
    }

    public static double getLastPowerFrontRight() {
        return lastPowerFrontRight;
    }

    public static void setLastPowerFrontRight(double lastPowerFrontRight) {
        RobotPIDF.lastPowerFrontRight = lastPowerFrontRight;
    }

    public static double getLastPowerBackLeft() {
        return lastPowerBackLeft;
    }

    public static void setLastPowerBackLeft(double lastPowerBackLeft) {
        RobotPIDF.lastPowerBackLeft = lastPowerBackLeft;
    }

    public static double getLastPowerBackRight() {
        return lastPowerBackRight;
    }

    public static void setLastPowerBackRight(double lastPowerBackRight) {
        RobotPIDF.lastPowerBackRight = lastPowerBackRight;
    }
}
