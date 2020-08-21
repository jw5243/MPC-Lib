package com.horse.mpclib.main;

import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.motion.IMotionProfile;
import com.horse.mpclib.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import com.horse.mpclib.lib.physics.LinearExtensionModel;
import com.horse.mpclib.lib.physics.MotorModel;

public class CascadeLinearlyExtendingRobotPractice extends Robot {
    private final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
    private final double spoolDiameter = 2d * 0.0254d; //m, 2 in
    private final double stageLength = 18d; //in
    private final int stageCount = 7;
    private LinearExtensionModel linearExtensionModel;

    private final double kS = 0d; //V
    private final double kV = 0.8d * (12d - kS) / 15d; //V s / in
    private final double kA = 0.22d; //V s^2 / in
    private final double kP = 10d; //V / in
    private final double kI = 2d; //V / (in s)
    private final double kD = 0d; //V s / in

    private double lastError = 0d;
    private double runningSum = 0d;

    private double setpoint = 15d / 2d;//5d; //in

    private IMotionProfile motionProfile;

    @Override
    public void init_debug() {
        super.init_debug();
        try {
            ComputerDebugger.send(MessageOption.CASCADE);
            ComputerDebugger.send(MessageOption.STAGE_COUNT.setSendValue(stageCount));
            ComputerDebugger.send(MessageOption.STAGE_LENGTH.setSendValue(stageLength));
            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue(90d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.GOBILDA_312_RPM, 2, 1d,
                        (motorPosition) -> mechanismWeight * spoolDiameter / 2d),
                spoolDiameter, 0.0025d, 0.002d
        );

        //linearExtensionModel.overridePosition(15d * 0.0254d);
        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0, 15, 15, 20);
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
            if(dt != 0) {
                double liftHeightInches = linearExtensionModel.getPosition() / 0.0254d;
                double error = setpoint - liftHeightInches;
                runningSum += error * dt;
                double output =
                        kS +
                        kP * error +
                        kI * runningSum +
                        kD * ((error - lastError) / dt - motionProfile.getVelocity());
                output = Math.min(12d, Math.max(-12d, output));

                lastError = error;

                linearExtensionModel.update(dt, output);
            }

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * 6d / 0.0254d) / 1000d));
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
            //ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * motionProfile.getVelocity()) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }
}
