package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.LinearExtensionModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

public class CascadeLinearlyExtendingRobot extends Robot {
    private final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
    private final double spoolDiameter = 2d * 0.0254d; //m, 2 in
    private final double stageLength = 18d; //in
    private final int stageCount = 7;
    private LinearExtensionModel linearExtensionModel;

    private final double kS = 4.76; //V
    private final double kV = (12d - kS) / 20d; //V s / in
    private final double kA = 0.22d; //V s^2 / in
    private final double kP = 1d;//4d; //V / in
    private final double kI = 0d; //V / (in s)
    private final double kD = 0d;//4d; //V s / in

    private double runningSum = 0d;
    private double lastError = 0d;

    private double setpoint = 15d; //in

    private IMotionProfile motionProfile;

    @Override
    public void init_debug() {
        super.init_debug();
        try {
            ComputerDebugger.send(MessageOption.CASCADE);
            ComputerDebugger.send(MessageOption.STAGE_COUNT.setSendValue(stageCount));
            ComputerDebugger.send(MessageOption.STAGE_LENGTH.setSendValue(stageLength));
            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue(0d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.GOBILDA_312_RPM, 2, 1d,
                        (motorPosition) -> mechanismWeight * spoolDiameter / 2d),
                spoolDiameter, 0.0025d, 0.002d
        );

        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 25d, 20d);
    }

    @Override
    public void start_debug() {
        super.start_debug();
        motionProfile.start();
    }

    @Override
    public void loop_debug() {
        try {
            super.loop_debug();
            double dt = getDt();
            double error = motionProfile.getPosition() - linearExtensionModel.getPosition() / 0.0254d;
            runningSum += error * dt;

            double output = kS +
                    kV * motionProfile.getVelocity() +
                    kA * motionProfile.getAcceleration() +
                    kP * error +
                    kI * runningSum +
                    kD * ((error - lastError) / dt - motionProfile.getVelocity());
            output = output < -12d ? -12d : output > 12d ? 12d : output;

            ComputerDebugger.send(MessageOption.LIFT_INPUT.setSendValue(output));

            linearExtensionModel.update(dt, output);
            lastError = error;

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * (stageCount - 1) / 0.0254d) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void sendMotionProfileData() {
        super.sendMotionProfileData();
        try {
            ComputerDebugger.send(MessageOption.LIFT_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() /** (stageCount - 1)*/ / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_VELOCITY.setSendValue((int)(1000d * linearExtensionModel.getVelocity() /** (stageCount - 1)*/ / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_ACCELERATION.setSendValue((int)(1000d * linearExtensionModel.getAcceleration() /** (stageCount - 1)*/ / 0.0254d) / 1000d));
            //ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * linearExtensionModel.getJerk() * 6d / 0.0254d) / 1000d));

            ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * motionProfile.getPosition()) /** (stageCount - 1)*/ / 1000d));
            //System.out.println(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) + "\t" + (int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d);
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    public static void main(String... args) {
        final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
        final double spoolDiameter = 0.55d * 0.0254d; //m, 0.55 in
        LinearExtensionModel linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.NEVEREST_3_7, 2, 1d,
                        (motorPosition) -> mechanismWeight * spoolDiameter / 2d),
                spoolDiameter, 0.0025d, 0.002d
        );

        final double dt = 0.001d; //s

        //final double kP = 26d; //V / in
        //final double kI = 1d; //V / (in s)
        //final double kD = 1d; //V s / in

        final double kS = 5.9d; //V
        final double kV = (12d - kS) / 22d; //V s / in
        final double kA = 0.001d; //V s^2 / in
        final double kP = 1d; //V / in
        final double kI = 0d; //V / (in s)
        final double kD = 1.2d; //V s / in

        double runningSum = 0d;
        double lastError = 0d;

        double setpoint = 15d; //in

        IMotionProfile motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 22d, 200d);

        System.out.println("t\tV in\ty\tv\ta");
        for(int i = 0; i < 2000; i++) {
            double timeStamp = i * dt;
            //double error = setpoint - linearExtensionModel.getPosition() / 0.0254d;
            double error = motionProfile.getPosition(timeStamp) - linearExtensionModel.getPosition() / 0.0254d;
            runningSum += error * dt;

            double output = kS +
                            kV * motionProfile.getVelocity(timeStamp) +
                            kA * motionProfile.getAcceleration(timeStamp) +
                            kP * error +
                            kI * runningSum +
                            kD * ((error - lastError) / dt - motionProfile.getVelocity(timeStamp));
            output = output < -12d ? 12d : output > 12d ? 12d : output;

            linearExtensionModel.update(dt, output);
            lastError = error;

            System.out.print((int) (timeStamp * 1000d) / 1000d + "\t");
            System.out.print(output + "\t");
            System.out.println(linearExtensionModel);
        }
    }
}
