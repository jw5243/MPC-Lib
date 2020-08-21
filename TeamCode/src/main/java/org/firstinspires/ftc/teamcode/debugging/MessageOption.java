package org.firstinspires.ftc.teamcode.debugging;

import org.firstinspires.ftc.teamcode.lib.geometry.Line2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public enum MessageOption {
    ROBOT_LOCATION(null, "ROBOT,"),
    KEY_POINT(Translation2d.class, "P,"),
    LOG_POINT(Translation2d.class, "LP,"),
    LINE(Line2d.class, "LINE,"),
    CLEAR_LOG_POINTS(null, "LogClear,"),
    POSITION(null, "Position,"),
    VELOCITY(null, "Velocity,"),
    ACCELERATION(null, "Acceleration,"),
    JERK(null, "Jerk,"),
    CLEAR_MOTION_PROFILE(null, "ProfileClear,"),
    LINEAR_POSITION(Double.class, "LinearPosition,"),
    STAGE_LENGTH(Double.class, "StageLength,"),
    STAGE_COUNT(Integer.class, "StageCount,"),
    CONTINUOUS(null, "Continuous"),
    CASCADE(null, "Cascade"),
    LIFT_POSITION(Double.class, "LiftPosition,"),
    LIFT_VELOCITY(Double.class, "LiftVelocity,"),
    LIFT_ACCELERATION(Double.class, "LiftAcceleration,"),
    LIFT_JERK(Double.class, "LiftJerk,"),
    LIFT_INPUT(Double.class, "LiftInput,"),
    TIME(null, "Time,"),
    STOP(null, null);

    private static final String END_MESSAGE_TAG = "%";

    private final Class<?> sendClass;
    private final String tag;
    private Object sendValue;

    MessageOption(final Class<?> sendClass, final String tag) {
        this.sendClass = sendClass;
        this.tag = tag;
    }

    public static String getEndMessageTag() {
        return END_MESSAGE_TAG;
    }

    public Class<?> getSendClass() {
        return sendClass;
    }

    public Object getSendValue() {
        return sendValue;
    }

    public MessageOption setSendValue(final Object sendValue) throws IllegalMessageTypeException {
        if(getSendClass() == null) {
            throw new IllegalMessageTypeException(this + " does not require any object to send. Do not call setSendValue(Object) method.");
        }

        if(sendValue == null) {
            return this;
        }

        if(sendValue.getClass() != getSendClass() && !getSendClass().isAssignableFrom(sendValue.getClass())) {
            throw new IllegalMessageTypeException(
                    "Cannot set send object to class of type " + sendValue.getClass().getSimpleName() + ". " + "Sending " + this +
                            " requires a send value of type " + getSendClass().getSimpleName());
        }

        this.sendValue = sendValue;
        return this;
    }

    @Override
    public String toString() {
        return ordinal() == ROBOT_LOCATION.ordinal() ? "SkystoneRobot Location" : ordinal() == KEY_POINT.ordinal() ? "Key Point" :
                ordinal() == LOG_POINT.ordinal() ? "Log Point" : ordinal() == LINE.ordinal() ? "Line" :
                        ordinal() == CLEAR_LOG_POINTS.ordinal() ? "Clear Log Points" : ordinal() == POSITION.ordinal() ? "Position" :
                                ordinal() == VELOCITY.ordinal() ? "Velocity" : ordinal() == ACCELERATION.ordinal() ? "Acceleration" :
                                        ordinal() == JERK.ordinal() ? "Jerk" :
                                                ordinal() == CLEAR_MOTION_PROFILE.ordinal() ? "Clear Motion Profile" : "Stop";
    }

    public String getTag() {
        return tag;
    }
}
