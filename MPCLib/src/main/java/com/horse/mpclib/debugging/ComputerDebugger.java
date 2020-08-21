package com.horse.mpclib.debugging;

import com.horse.mpclib.lib.geometry.Line2d;
import com.horse.mpclib.lib.geometry.Translation2d;
import com.horse.mpclib.lib.util.TimeUnits;
import com.horse.mpclib.lib.util.TimeUtil;
import com.horse.mpclib.examples.Robot;
import com.horse.mpclib.examples.RobotMPC;

import java.text.DecimalFormat;

public class ComputerDebugger {
    private static UdpServer udpServer;
    private static StringBuilder messageBuilder;
    private static DecimalFormat formatter;
    private static Robot robot;

    public static void init(final Robot robot) {
        setRobot(robot);
        setMessageBuilder(new StringBuilder());
        setFormatter(new DecimalFormat("#.00"));
        setUdpServer(new UdpServer(15026));
        new Thread(getUdpServer()).start();
    }

    public static void main(String[] args) {
        Robot robot = new RobotMPC();
        init(robot);
        robot.init_debug();

        send(MessageOption.CLEAR_LOG_POINTS);
        send(MessageOption.CLEAR_MOTION_PROFILE);
        sendMessage();

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.start_debug();
        while(true) {
            try {
                robot.loop_debug();
                robot.sendMotionProfileData();

                Thread.sleep(10);

                send(MessageOption.ROBOT_LOCATION);
                send(MessageOption.LOG_POINT.setSendValue(robot.getFieldPosition().getTranslation()));
                sendMessage();
            } catch (InterruptedException | IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }
    }

    public static void send(final MessageOption messageOption) {
        if(messageOption == null) {
            return;
        }

        getMessageBuilder().append(messageOption.getTag());

        if(messageOption.ordinal() == MessageOption.ROBOT_LOCATION.ordinal()) {
            getMessageBuilder().append(getRobot().getFieldPosition());
        } else if(messageOption.ordinal() == MessageOption.KEY_POINT.ordinal()) {
            final Translation2d keyPoint = (Translation2d)(messageOption.getSendValue());
            getMessageBuilder().append(keyPoint);
        } else if(messageOption.ordinal() == MessageOption.LOG_POINT.ordinal()) {
            final Translation2d logPoint = (Translation2d)(messageOption.getSendValue());
            getMessageBuilder().append(logPoint);
        } else if(messageOption.ordinal() == MessageOption.LINE.ordinal()) {
            final Line2d line = (Line2d)(messageOption.getSendValue());
            getMessageBuilder().append(line);
        } else if(messageOption.ordinal() == MessageOption.CLEAR_LOG_POINTS.ordinal()) {
            //
        } else if(messageOption.ordinal() == MessageOption.POSITION.ordinal()) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(getRobot().getFieldPosition());
        } else if(messageOption.ordinal() == MessageOption.VELOCITY.ordinal()) {
            //getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentAngularVelocity());
        } else if(messageOption.ordinal() == MessageOption.ACCELERATION.ordinal()) {
            //getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentAcceleration());
        } else if(messageOption.ordinal() == MessageOption.JERK.ordinal()) {
            //getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentJerk());
        } else if(messageOption.ordinal() == MessageOption.CLEAR_MOTION_PROFILE.ordinal()) {
            //
        } else if(messageOption.ordinal() == MessageOption.LINEAR_POSITION.ordinal()) {
            final double position = (Double)(messageOption.getSendValue());
            getMessageBuilder().append(position);
        } else if(messageOption.ordinal() == MessageOption.STAGE_LENGTH.ordinal()) {
            final double length = (Double)(messageOption.getSendValue());
            getMessageBuilder().append(length);
        } else if(messageOption.ordinal() == MessageOption.STAGE_COUNT.ordinal()) {
            final int stageCount = (Integer)(messageOption.getSendValue());
            getMessageBuilder().append(stageCount);
        } else if(messageOption.ordinal() == MessageOption.CASCADE.ordinal()) {

        } else if(messageOption.ordinal() == MessageOption.CONTINUOUS.ordinal()) {

        } else if(messageOption.equals(MessageOption.LIFT_POSITION)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append((double)(messageOption.getSendValue()));
        } else if(messageOption.equals(MessageOption.LIFT_VELOCITY)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append((double)(messageOption.getSendValue()));
        } else if(messageOption.equals(MessageOption.LIFT_ACCELERATION)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append((double)(messageOption.getSendValue()));
        } else if(messageOption.equals(MessageOption.LIFT_JERK)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append((double)(messageOption.getSendValue()));
        } else if(messageOption.equals(MessageOption.LIFT_INPUT)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append((double)(messageOption.getSendValue()));
        } else if(messageOption.equals(MessageOption.TIME)) {
            getMessageBuilder().append(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
        } else {
            getUdpServer().close();
        }

        getMessageBuilder().append(MessageOption.getEndMessageTag());
    }

    public static void sendMessage() {
        getMessageBuilder().append("CLEAR,%");
        getUdpServer().addMessage(getMessageBuilder().toString());
        setMessageBuilder(new StringBuilder());
    }

    public static UdpServer getUdpServer() {
        return udpServer;
    }

    public static void setUdpServer(UdpServer udpServer) {
        ComputerDebugger.udpServer = udpServer;
    }

    public static StringBuilder getMessageBuilder() {
        return messageBuilder;
    }

    public static void setMessageBuilder(StringBuilder messageBuilder) {
        ComputerDebugger.messageBuilder = messageBuilder;
    }

    public static DecimalFormat getFormatter() {
        return formatter;
    }

    public static void setFormatter(DecimalFormat formatter) {
        ComputerDebugger.formatter = formatter;
    }

    public static Robot getRobot() {
        return robot;
    }

    public static void setRobot(Robot robot) {
        ComputerDebugger.robot = robot;
    }
}
