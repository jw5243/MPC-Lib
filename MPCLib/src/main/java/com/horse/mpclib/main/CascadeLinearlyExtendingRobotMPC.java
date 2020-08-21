package com.horse.mpclib.main;

import org.ejml.simple.SimpleMatrix;
import com.horse.mpclib.debugging.ComputerDebugger;
import com.horse.mpclib.debugging.IllegalMessageTypeException;
import com.horse.mpclib.debugging.MessageOption;
import com.horse.mpclib.lib.control.LinearExtensionLQR;
import com.horse.mpclib.lib.drivers.Motor;
import com.horse.mpclib.lib.motion.IMotionProfile;
import com.horse.mpclib.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import com.horse.mpclib.lib.physics.LinearExtensionModel;
import com.horse.mpclib.lib.physics.MotorModel;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;

public class CascadeLinearlyExtendingRobotMPC extends Robot {
    private final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
    private final double spoolDiameter = 2d * 0.0254d; //m, 2 in
    private final double stageLength = 18d; //in
    private final int stageCount = 7;
    private LinearExtensionModel linearExtensionModel;
    private LinearExtensionLQR mpc;

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

        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 20d, 10d);

        mpc = new LinearExtensionLQR(linearExtensionModel, new SimpleMatrix(2, 1, true, new double[] {
                0d, 0d
        }), new SimpleMatrix(2, 1, true, new double[] {
                setpoint * 0.0254d, 0d
        }));

        mpc.runLQR();
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

            double input = mpc.getOptimalInput((int)(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) / LinearExtensionLQR.getDt()),
                    new SimpleMatrix(2, 1, true, new double[] {
                    linearExtensionModel.getPosition(), linearExtensionModel.getVelocity()
            }), new SimpleMatrix(2, 1, true, new double[] {
                    motionProfile.getPosition() * 0.0254d, motionProfile.getVelocity() * 0.0254d
            }));

            linearExtensionModel.update(dt, input);

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * 6d / 0.0254d) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void sendMotionProfileData() {
        super.sendMotionProfileData();
        try {
            ComputerDebugger.send(MessageOption.LIFT_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * 6d / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_VELOCITY.setSendValue((int)(1000d * linearExtensionModel.getVelocity() * 6d / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_ACCELERATION.setSendValue((int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * motionProfile.getPosition() * 6d) / 1000d));

            //System.out.println(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) + "\t" + (int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d);
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }
}
