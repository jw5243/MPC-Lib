package com.horse.mpclib.examples;

import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.motion.IMotionProfile;
import com.horse.mpclib.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import com.horse.mpclib.lib.physics.LinearExtensionModel;
import com.horse.mpclib.lib.physics.MotorModel;

import java.util.function.DoubleToIntFunction;
import java.util.function.IntToDoubleFunction;

public class ContinuousLinearlyExtendingRobot extends Robot {
    private final double slideWeight = 4.448d * 1d; //N, 1 lbs
    private final double mechanismWeight = 4.448d * 1.5d; //N, 1 lbs
    private final double spoolDiameter = 2.25d * 0.0254d; //m, 1 in
    private final double stageLength = 18d; //in
    private final int stageCount = 6;
    private LinearExtensionModel linearExtensionModel;

    private final double kS = 6d; //V
    private final double kV = (12d - kS) / 50d; //V s / in
    private final double kA = 0d;//0.2d; //V s^2 / in
    private final double kP = 5d; //V / in
    private final double kI = 0d; //V / (in s)
    private final double kD = 0d; //V s / in

    private double runningSum = 0d;
    private double lastError = 0d;

    private double setpoint = 15 * 6d; //in

    private IMotionProfile motionProfile;

    @Override
    public void init_debug() {
        super.init_debug();
        try {
            ComputerDebugger.send(MessageOption.CONTINUOUS);
            ComputerDebugger.send(MessageOption.STAGE_COUNT.setSendValue(stageCount));
            ComputerDebugger.send(MessageOption.STAGE_LENGTH.setSendValue(stageLength));
            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue(0d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        DoubleToIntFunction currentStage = (motorPosition) -> (int) (1 + MotorModel.getLinearPosition(motorPosition, spoolDiameter) / (0.0254d * stageLength));
        IntToDoubleFunction effectiveLoad = (stage) -> (stage < 1 ? 0d : mechanismWeight + slideWeight / 2d +
                (stage < stageCount ? slideWeight * (stage - 1) : slideWeight * (stageCount - 1))) * spoolDiameter / 2d;

        linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.NEVEREST_3_7, 2, 1d,
                        (motorPosition) -> effectiveLoad.applyAsDouble(currentStage.applyAsInt(motorPosition))),
                spoolDiameter, 0.0025d, 0.002d
        );

        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 60d, 100d);
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

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int) (1000d * linearExtensionModel.getPosition() / 0.0254d) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void sendMotionProfileData() {
        super.sendMotionProfileData();
        try {
            ComputerDebugger.send(MessageOption.LIFT_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_VELOCITY.setSendValue((int)(1000d * linearExtensionModel.getVelocity() / 0.0254d) / 1000d));
            //ComputerDebugger.send(MessageOption.LIFT_ACCELERATION.setSendValue((int)(1000d * linearExtensionModel.getAcceleration() / 0.0254d) / 1000d));

            ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * motionProfile.getPosition()) / 1000d));
            //System.out.println(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) + "\t" + (int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d);
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }
}
