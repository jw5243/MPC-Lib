package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;

import java.util.function.DoubleUnaryOperator;

/**
 * This {@code class} simulates and stores the basic components of a motor, including torque, speed,
 * and currently four types of friction. This model of the motor has currently been used to simulate
 * voltage input of the motor, rather than PWM type inputs based on an input power that is used in
 * FTC. Nevertheless, the difference between these two methods of powering a motor are assumed to be
 * close enough to the same that this simulation is negligibly different.
 *
 * At the very least, any controller based on a model should be robust, and such a difference in motor
 * powering should be compensated by the control system implemented.
 *
 * It is also important to note that this class uses International Standard (SI) units whenever possible.
 * The only exception is for wheel velocities as conversions between RPM and rad / s is common.
 *
 * @see LinearExtensionModel
 */
public class MotorModel {
    /**
     * This value converts a value in units of RPM to the corresponding value in radians per second
     * upon multiplication. Analogously, division by this number converts radians per second to RPM.
     */
    private static final double RPM_TO_RAD_PER_SECOND = Math.PI / 30d; //rad min / (rot s)

    /**
     * This value represents the amount of gearing to slow down and increase the torque of the motor.
     * Motors without any gearbox run at very high RPM, so gear ratios are an integral part of any
     * motorized mechanism.
     */
    private final double gearRatio;

    /**
     * The nominal voltage is most commonly 12 V in FTC, representing the voltage at which the motor
     * is tagged to be powered.
     */
    private final double nominalVoltage; //V

    /**
     * This value represents the max amount of torque output from the motor. More precisely, the stall
     * torque is the amount of torque the motor supplies when running at zero speed. The stall torque
     * value stored in this variable is that of the output shaft of the gearings of the motor, so
     * the torque here is that from the motor specifications multiplied by the external gear ratio.
     */
    private final double stallTorque; //N m

    /**
     * This value denotes the amount of current draw by the motor when stalled. Specifically, the motor
     * is stalled when applying voltage does not cause the motor to start moving.
     */
    private final double stallCurrent; //A

    /**
     * This value represents the current draw when the motor is running at full speed with no load on
     * the shaft.
     */
    private final double freeCurrent; //A

    /**
     * This value represents how fast the motor runs when no load is placed on the output shaft. Thus,
     * the free speed is the speed in rad / s after the gearing is applied.
     */
    private final double freeSpeed; //rad / s

    /**
     * This value represents the ratio between the power of the output shaft versus the internal rotor.
     * This value will generally be below 0.8 (80%), especially for brushed motors.
     */
    private final double efficiency;

    /**
     * Motors must have some sort of resistor to ensure that the current draw is not excessively high.
     * The resistance relates the voltage input to the current draw via the equation V = IR.
     */
    private final double resistance; //ohms

    /**
     * This value is the conversion factor from angular velocity (rad / s) to voltage (V). The
     * determination of this value depends on the specifications of the motor that is being described.
     * Thus, based on parameters supplied to the constructor, this value is automatically determined
     * for convenience.
     */
    private final double kV; //V rad / s

    /**
     * This value converts current (A) to torque (N m). This value is determined from the parameters
     * supplied in the constructor, and thereby calculated automatically for convenience. Like kV,
     * kT depends on motor specifications.
     *
     * @see #getkV()
     */
    private final double kT; //N m / A

    /**
     * This operator determines the amount of inertia the motor must overcome when running. Specifically,
     * we are using a {@code DoubleUnaryOperator} as a way to allow for different inertia values for
     * different angular displacements. This value is likely to be constant, so unchanging for varying
     * angular displacements. The {@code weightAppliedTorque} is more likely to be varying.
     *
     * @see DoubleUnaryOperator
     * @see #getCurrentAngularPosition()
     * @see #getWeightAppliedTorque()
     */
    private final DoubleUnaryOperator inertia; //kg m^2

    /**
     * This operator determines the amount of 'weight' that the motor must overcome when running. Based
     * on the current angular displacement of the motor as input for this operator, what should return
     * is the amount of torque induced by the weight of the mechanism for which the motor is running.
     * The value being returned here need not be the torque from weight, but that is the most likely
     * use of this parameter, and thus we call it the {@code weightAppliedTorque}.
     *
     * @see #getCurrentAngularPosition()
     */
    private final DoubleUnaryOperator weightAppliedTorque; //N m

    //Define friction coefficients

    /**
     * This value is the amount of torque required to start spinning the motor (minus backdrive torque).
     * This value is should be greater than or equal to the {@see coulombFriction} coefficient.
     */
    private final double staticFriction; //N m
    private final double coulombFriction; //N m
    private final double viscousFriction; //N s
    private final double stribeckPower;
    private final double stribeckVelocity; //rad / s

    private double currentAngularPosition; //rad
    private double currentAngularVelocity; //rad / s
    private double lastAngularAcceleration; //rad / s^2

    /**
     * Defines the parameters of the motors based on values that can easily be found on motor spec
     * charts, in addition to the structure that is being built. All the parameters in the constructor
     * completely defines the basic motor model.
     *
     * @param gearRatio      Gearing on the motor to increase the torque output and reduce the max speed
     *                       of the output shaft. FTC motors commonly come with an internal gearbox
     *                       that will adjust the free speed already, in which case, set this value
     *                       to the external gear ratio that is compounded to the motor for further
     *                       adjustments of torque and speed. A number greater than one for this parameter
     *                       suggests an increase and torque and reduction in speed.
     * @param nominalVoltage The theoretical max voltage of the motor, which in FTC should always be 12 V.
     * @param stallTorque    The amount of torque supplied if there is no internal nor external gearbox
     *                       for the motor. The output shaft torque is automatically adjusted base on
     *                       the {@code gearRatio} parameter.
     * @param stallCurrent   The amount of current draw from the motor if the motor is stalled.
     * @param freeCurrent    The amount of current draw from the motor if there is nothing attached
     *                       to the motor, thus allowing it to spin freely.
     * @param freeSpeed      The RPM of the motor in the case of nothing being attached to the motor
     *                       and being run at full speed.
     * @param efficiency     The amount of torque output loss due to gears, frictions, and other
     *                       environmental disturbances that cannot exactly be accounted for. Most
     *                       of the FTC motors use experimental values for the stall torque, so the
     *                       efficiency can be assumed to be 100% unless there is an external gearbox.
     *                       A generally good efficiency coefficient is 80%.
     * @param inertia        The amount of "stuff" preventing the motor from running at its free speed.
     *                       As this value can vary depending on the angular position of the motor
     *                       output shaft, this parameter is made a {@code DoubleSupplier}, enabling
     *                       custom inputs for the inertia value based on the angular rotation of the
     *                       motor output shaft.
     *
     * @see DoubleUnaryOperator
     */
    public MotorModel(double gearRatio, double nominalVoltage, double stallTorque,
                      double stallCurrent, double freeCurrent, double freeSpeed, double efficiency,
                      DoubleUnaryOperator inertia, DoubleUnaryOperator weightAppliedTorque, double staticFriction,
                      double coulombFriction, double viscousFriction, double stribeckPower, double stribeckVelocity) {
        this.gearRatio = gearRatio;
        this.nominalVoltage = nominalVoltage;
        this.stallTorque = stallTorque * gearRatio * efficiency;
        this.stallCurrent = stallCurrent;
        this.freeCurrent = freeCurrent;
        this.freeSpeed = freeSpeed * getRpmToRadPerSecond();
        this.efficiency = efficiency;

        this.resistance = getNominalVoltage() / getStallCurrent();

        this.kV = (getNominalVoltage() - getResistance() * getFreeCurrent()) / getFreeSpeed();
        this.kT = getStallTorque() / getStallCurrent();

        this.inertia = inertia;
        this.weightAppliedTorque = weightAppliedTorque;

        this.staticFriction = staticFriction;
        this.coulombFriction = coulombFriction;
        this.viscousFriction = viscousFriction;
        this.stribeckPower = stribeckPower;
        this.stribeckVelocity = stribeckVelocity;

        setCurrentAngularPosition(0d);
        setCurrentAngularVelocity(0d);
        setLastAngularAcceleration(0d);
    }

    public static MotorModel generateMotorModel(Motor motorType, int motorCount, double externalGearRatio, DoubleUnaryOperator inertia, DoubleUnaryOperator weightAppliedTorque) {
        return new MotorModel(externalGearRatio, 12d, motorType.getStallTorque() * motorCount,
                motorType.getStallCurrent(), motorType.getFreeCurrent(), motorType.getRPM(), 1d,
                inertia, weightAppliedTorque, 3E-3d * motorCount, 2E-3d * motorCount, 1E-4d,
                0.05d, 25d);
    }

    public static MotorModel generateMotorModel(Motor motorType, double externalGearRatio, DoubleUnaryOperator inertia, DoubleUnaryOperator weightAppliedTorque) {
        return generateMotorModel(motorType, 1, externalGearRatio, inertia, weightAppliedTorque);
    }

    public static MotorModel generateMotorModel(Motor motorType, DoubleUnaryOperator inertia, DoubleUnaryOperator weightAppliedTorque) {
        return generateMotorModel(motorType, 1, 1d, inertia, weightAppliedTorque);
    }

    public static MotorModel generateMotorModel(Motor motorType, int motorCount, double externalGearRatio, DoubleUnaryOperator weightAppliedTorque) {
        return new MotorModel(externalGearRatio, 12d, motorType.getStallTorque() * motorCount,
                motorType.getStallCurrent(), motorType.getFreeCurrent(), motorType.getRPM(), 1d,
                (motorPosition) -> motorType.getBackdriveTorque(), weightAppliedTorque, 3E-3d * motorCount,
                2E-3d * motorCount, 1E-4d, 0.05d, 25d);
    }

    public static MotorModel generateMotorModel(Motor motorType, double externalGearRatio, DoubleUnaryOperator weightAppliedTorque) {
        return generateMotorModel(motorType, 1, externalGearRatio, weightAppliedTorque);
    }

    public static MotorModel generateMotorModel(Motor motorType, DoubleUnaryOperator weightAppliedTorque) {
        return generateMotorModel(motorType, 1, 1d, weightAppliedTorque);
    }

    public static void main(String... args) {
        MotorModel motorModel = new MotorModel(
                20d, 12d, 3.36d, 166d,
                1.3d, 5880d, 0.8d, (motorPosition) -> 2d, (motorPosition) -> 0d,
                3E-3, 2E-3, 1E-4, 0.05d, 25d);

        System.out.println("Time\tθ\tω\tα");
        final double dt = 0.001d;
        for(int i = 1; i < 500; i++) {
            motorModel.update(dt, 12d);
            System.out.print((int) (dt * i * 1000d) / 1000d + "\t");
            System.out.println(motorModel);
        }
    }

    /**
     *
     *
     * @param dt
     * @param voltageInput
     */
    public void update(double dt, double voltageInput) {
        update(dt, voltageInput, 0d);
    }

    /**
     * Updates the motor model by calculating the theoretically angular acceleration/torque generated
     * by the motor for a given voltage input. Thereafter, we can calculate the angular velocity and
     * angular displacement (position) via riemann integration or simply the kinematic equations for
     * constant angular acceleration. We can use the constant acceleration kinematic equations since
     * we assume for a small time dt, the angular acceleration of the motor is approximately constant.
     *
     * @param dt The elapsed time in seconds since the last time this {@code method} was called.
     * @param voltageInput The amount of volts supplied to the motor model.
     * @param externalFriction The frictional torque caused by external sources, such as that coming from
     *                         surface contact of a mechanism run by a motor.
     */
    public void update(double dt, double voltageInput, double externalFriction) {
        if(dt == 0d) {
            return;
        }

        voltageInput = voltageInput > getNominalVoltage() ? getNominalVoltage() : voltageInput < -getNominalVoltage() ? -getNominalVoltage() : voltageInput;
        setLastAngularAcceleration(calculateAngularAcceleration(voltageInput, externalFriction));
        setCurrentAngularVelocity(getCurrentAngularVelocity() + getLastAngularAcceleration() * dt);
        setCurrentAngularPosition(getCurrentAngularPosition() + getCurrentAngularVelocity() * dt);

        //Assume that there is a physical stop preventing the system from backdriving past the 0
        //position in the opposite direction of desired motion.
        if(getCurrentAngularPosition() < 0d) {
            setCurrentAngularPosition(0d);
        }
    }

    /**
     * Determine the theoretical output torque of the motor.
     *
     * @param voltageInput
     * @return
     */
    public double calculateTorque(double voltageInput) {
        return calculateTorque(voltageInput, 0d);
    }

    /**
     * Determine the theoretical output torque of the motor in the case of additional friction aside
     * from that inside the motor itself.
     *
     * @param voltageInput
     * @return
     */
    public double calculateTorque(double voltageInput, double externalFriction) {
        double torque = getkT() * getEfficiency() * getGearRatio() * (voltageInput - getkV() * getCurrentAngularVelocity() * getGearRatio()) / getResistance();
        double frictionTorque = getFrictionTorque() + externalFriction;
        return Math.abs(Math.signum(torque) - Math.signum(torque - frictionTorque)) == 2d ? 0d :
                torque - frictionTorque - (getWeightAppliedTorque() == null ? 0d : getWeightAppliedTorque().applyAsDouble(getCurrentAngularPosition()));
    }

    /**
     * Determine the theoretical acceleration of the motor output shaft.
     *
     * @param voltageInput
     * @return
     */
    public double calculateAngularAcceleration(double voltageInput) {
        return calculateAngularAcceleration(voltageInput, 0d);
    }

    /**
     * Determine the theoretical acceleration of the motor output shaft in the case of additional
     * friction aside from that inside the motor shaft.
     *
     * @param voltageInput
     * @return
     */
    public double calculateAngularAcceleration(double voltageInput, double externalFriction) {
        return calculateTorque(voltageInput, externalFriction) / getInertia().applyAsDouble(getCurrentAngularPosition());
    }

    public static double getLinearPosition(double currentAngularPosition, double rotationDiameter) {
        return currentAngularPosition * rotationDiameter / 2d;
    }

    /**
     * Converts angular displacement into linear displacement based on a given rotation diameter.
     *
     * @param rotationDiameter
     * @return
     */
    public double getLinearPosition(double rotationDiameter) {
        return getCurrentAngularPosition() * rotationDiameter / 2d;
    }

    /**
     * Converts angular velocity into linear velocity based on a given rotation diameter.
     *
     * @param rotationDiameter
     * @return
     */
    public double getLinearVelocity(double rotationDiameter) {
        return getCurrentAngularVelocity() * rotationDiameter / 2d;
    }

    /**
     * Converts angular acceleration into linear acceleration based on a given rotation diameter.
     *
     * @param rotationDiameter
     * @return
     */
    public double getLinearAcceleration(double rotationDiameter) {
        return getLastAngularAcceleration() * rotationDiameter / 2d;
    }

    public double getFrictionTorque() {
        return Math.signum(getCurrentAngularVelocity()) != 0 ? Math.signum(getCurrentAngularVelocity()) * (getCoulombFriction() +
                (getStaticFriction() - getCoulombFriction()) * Math.exp(-Math.pow(Math.abs(getCurrentAngularVelocity() /
                        getStribeckVelocity()), getStribeckPower())) + getViscousFriction() * Math.abs(getCurrentAngularVelocity()))
                : Math.signum(getStaticFriction()) * getStaticFriction();
    }

    public double getFrictionTorque(double angularVelocity) {
        return Math.signum(angularVelocity) != 0 ? Math.signum(angularVelocity) * (getCoulombFriction() +
                (getStaticFriction() - getCoulombFriction()) * Math.exp(-Math.pow(Math.abs(angularVelocity /
                        getStribeckVelocity()), getStribeckPower())) + getViscousFriction() * Math.abs(angularVelocity))
                : Math.signum(getStaticFriction()) * getStaticFriction();
    }

    @Override
    public String toString() {
        //return "Motor Model {θ: " + getCurrentAngularPosition() + ", ω: " +
        //        getCurrentAngularVelocity() + ", α: " + getLastAngularAcceleration() + "}";
        return getCurrentAngularPosition() + "\t" + getCurrentAngularVelocity() + "\t" + getLastAngularAcceleration();
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getResistance() {
        return resistance;
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

    public double getFreeSpeed() {
        return freeSpeed;
    }

    public double getEfficiency() {
        return efficiency;
    }

    public double getkV() {
        return kV;
    }

    public double getkT() {
        return kT;
    }

    public DoubleUnaryOperator getInertia() {
        return inertia;
    }

    public double getCurrentAngularVelocity() {
        return currentAngularVelocity;
    }

    public void setCurrentAngularVelocity(double currentAngularVelocity) {
        this.currentAngularVelocity = currentAngularVelocity;
    }

    public double getCurrentAngularPosition() {
        return currentAngularPosition;
    }

    public void setCurrentAngularPosition(double currentAngularPosition) {
        this.currentAngularPosition = currentAngularPosition;
    }

    public static double getRpmToRadPerSecond() {
        return RPM_TO_RAD_PER_SECOND;
    }

    public double getLastAngularAcceleration() {
        return lastAngularAcceleration;
    }

    public void setLastAngularAcceleration(double lastAngularAcceleration) {
        this.lastAngularAcceleration = lastAngularAcceleration;
    }

    public double getStaticFriction() {
        return staticFriction;
    }

    public double getCoulombFriction() {
        return coulombFriction;
    }

    public double getViscousFriction() {
        return viscousFriction;
    }

    public double getStribeckPower() {
        return stribeckPower;
    }

    public double getStribeckVelocity() {
        return stribeckVelocity;
    }

    public DoubleUnaryOperator getWeightAppliedTorque() {
        return weightAppliedTorque;
    }
}
