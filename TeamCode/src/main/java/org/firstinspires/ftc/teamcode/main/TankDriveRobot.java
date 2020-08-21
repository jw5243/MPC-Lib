package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.control.TankDriveILQR;
import org.firstinspires.ftc.teamcode.lib.control.TankDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.TankRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.control.TankRunnableMPC;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;
import org.firstinspires.ftc.teamcode.lib.physics.TankDriveModel;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public abstract class TankDriveRobot extends Robot {
    private static SimpleMatrix tankState;
    private static SimpleMatrix tankInput;

    private static TankDriveModel tankDriveModel;
    private static TankDriveILQR tankDriveILQR;
    private TankRunnableLQR tankRunnableLQR;

    private static TankDriveMPC tankDriveMPC;
    private TankRunnableMPC tankRunnableMPC;

    @Override
    public void init_debug() {
        super.init_debug();
        setTankState(new SimpleMatrix(5, 1, true, new double[] {
                getInitialState().get(0), getInitialState().get(2), getInitialState().get(4), 0, 0
        }));

        setTankInput(new SimpleMatrix(2, 1, true, new double[] {
                0d, 0d
        }));

        setTankDriveModel(new TankDriveModel(15.75d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2, 0.5613d,
                0.1d / 2, 16d * 0.0254d, MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null)));
    }

    @Override
    public void loop_debug() {
        setDt(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
        setTankState(getTankDriveModel().simulateNonlinear(getTankState(), getTankInput(), getDt()));
    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d(getTankState().get(0) / 0.0254d, getTankState().get(1) / 0.0254d, new Rotation2d(getTankState().get(2), false));
    }

    public static TankDriveModel getTankDriveModel() {
        return tankDriveModel;
    }

    public static void setTankDriveModel(TankDriveModel tankDriveModel) {
        TankDriveRobot.tankDriveModel = tankDriveModel;
    }

    public static TankDriveILQR getTankDriveILQR() {
        return tankDriveILQR;
    }

    public static void setTankDriveILQR(TankDriveILQR tankDriveILQR) {
        TankDriveRobot.tankDriveILQR = tankDriveILQR;
    }

    public TankRunnableLQR getTankRunnableLQR() {
        return tankRunnableLQR;
    }

    public void setTankRunnableLQR(TankRunnableLQR tankRunnableLQR) {
        this.tankRunnableLQR = tankRunnableLQR;
    }

    public static SimpleMatrix getTankState() {
        return tankState;
    }

    public static void setTankState(SimpleMatrix tankState) {
        TankDriveRobot.tankState = tankState;
    }

    public static SimpleMatrix getTankInput() {
        return tankInput;
    }

    public static void setTankInput(SimpleMatrix tankInput) {
        TankDriveRobot.tankInput = tankInput;
    }

    public static TankDriveMPC getTankDriveMPC() {
        return tankDriveMPC;
    }

    public static void setTankDriveMPC(TankDriveMPC tankDriveMPC) {
        TankDriveRobot.tankDriveMPC = tankDriveMPC;
    }

    public TankRunnableMPC getTankRunnableMPC() {
        return tankRunnableMPC;
    }

    public void setTankRunnableMPC(TankRunnableMPC tankRunnableMPC) {
        this.tankRunnableMPC = tankRunnableMPC;
    }
}
