package com.horse.mpclib.lib.physics;

import java.text.DecimalFormat;

public class RingSimulator {
    private static final double GRAVITATIONAL_ACCELERATION = 32.174d * 12d; //in/s^2
    private static final DecimalFormat formatter = new DecimalFormat("#.00");

    private double dragFactor;

    private double x;
    private double y;
    private double z;

    private double vx;
    private double vy;
    private double vz;

    public RingSimulator(double x0, double y0, double z0, double vx0, double vy0, double vz0, double dragFactor) {
        setX(x0);
        setY(y0);
        setZ(z0);
        setVx(vx0);
        setVy(vy0);
        setVz(vz0);
        setDragFactor(dragFactor);
    }

    public void simulate(double dt) {
        double speed = Math.sqrt(getVx() * getVx() + getVy() * getVy() + getVz() + getVz());
        double deltaVz = -getGravitationalAcceleration() * dt;

        setVz(getVz() + deltaVz);

        setX(getX() + getVx() * dt);
        setY(getY() + getVy() * dt);
        setZ(getZ() + getVz() * dt);
    }

    @Override
    public String toString() {
        //return getFormatter().format(getX()) + "\t" + getFormatter().format(getY()) + "\t" + getFormatter().format(getZ());
        return "{" + getFormatter().format(getX()) + "," + getFormatter().format(getY()) + "," + getFormatter().format(getZ()) + "}";
    }

    public static void main(String... args) {
        RingSimulator simulator = new RingSimulator(0d, 0d, 0d, 50d, 70d, 200d, 0d);
        //System.out.println(simulator);
        System.out.print("{");
        System.out.print(simulator);
        int timeSteps = 100;
        double dt = 0.01d;
        for(int i = 0; i < timeSteps; i++) {
            simulator.simulate(dt);
            //System.out.println(((int)(i * dt * 100d) / 100d) + "\t" + simulator.toString());
            System.out.print("," + simulator.toString());
        }

        System.out.print("}");
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public double getVx() {
        return vx;
    }

    public void setVx(double vx) {
        this.vx = vx;
    }

    public double getVy() {
        return vy;
    }

    public void setVy(double vy) {
        this.vy = vy;
    }

    public double getVz() {
        return vz;
    }

    public void setVz(double vz) {
        this.vz = vz;
    }

    public static double getGravitationalAcceleration() {
        return GRAVITATIONAL_ACCELERATION;
    }

    public static DecimalFormat getFormatter() {
        return formatter;
    }

    public double getDragFactor() {
        return dragFactor;
    }

    public void setDragFactor(double dragFactor) {
        this.dragFactor = dragFactor;
    }
}
