package org.firstinspires.ftc.teamcode.lib.drivers;

public enum Motor {
    GOBILDA_1150_RPM(1150d, 145.6d, 0.7697091498333297d,
            9.2d, 0.25d, getDefaultBackdriveTorquePerGearing() * 5.2d),
    GOBILDA_435_RPM(435d, 383.6d, 1.836003476666658d,
            9.2d, 0.25d, getDefaultBackdriveTorquePerGearing() * 13.7d),
    GOBILDA_312_RPM(312d, 537.6d, 2.3868045196666556d,
            9.2d, 0.25d, getDefaultBackdriveTorquePerGearing() * 19.2d),
    GOBILDA_223_RPM(223d, 753.2d, 3.742622471666649d,
            9.2d, 0.25d, getDefaultBackdriveTorquePerGearing() * 26.9d),
    NEVEREST_3_7(1780d, 103.6d, 0.17d * 3.7d,
            9.801d, 0.355d, getDefaultBackdriveTorquePerGearing() * 3.7d),
    NEVEREST_20(340d, 537.6d, 0.17d * 20d,
            9.801d, 0.355d, getDefaultBackdriveTorquePerGearing() * 20d),
    REV_CORE_HEX(125d, 288d, 3.2d,
            4.4d, 0d, getDefaultBackdriveTorquePerGearing() * 72d);

    private static final double DEFAULT_BACKDRIVE_TORQUE_PER_GEARING = 0.01694772439999992d / 3.7d;

    private final double rpm;
    private final double encoderTicksPerRevolution;
    private final double stallTorque; //N m
    private final double stallCurrent; //A
    private final double freeCurrent; //A
    private final double backdriveTorque; //N m

    Motor(double rpm, double encoderTicksPerRevolution, double stallTorque, double stallCurrent,
          double freeCurrent, double backdriveTorque) {
        this.rpm = rpm;
        this.encoderTicksPerRevolution = encoderTicksPerRevolution;
        this.stallTorque = stallTorque;
        this.stallCurrent = stallCurrent;
        this.freeCurrent = freeCurrent;
        this.backdriveTorque = backdriveTorque;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return rpm;
    }

    public double getEncoderTicksPerRevolution() {
        return encoderTicksPerRevolution;
    }

    public double getStallTorque() {
        return stallTorque;
    }

    public double getStallCurrent() {
        return stallCurrent;
    }

    public double getFreeCurrent() {
        return freeCurrent;
    }

    public double getBackdriveTorque() {
        return backdriveTorque;
    }

    public static double getDefaultBackdriveTorquePerGearing() {
        return DEFAULT_BACKDRIVE_TORQUE_PER_GEARING;
    }
}
