package com.horse.mpclib.lib.geometry;

import java.text.DecimalFormat;

public class Circle2d extends Translation2d {
    private double radius;

    public Circle2d(double x, double y, double radius) {
        super(x, y);
        setRadius(radius);

    }

    public Circle2d(Translation2d translation, double radius) {
        super(translation);
        setRadius(radius);
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    @Override
    public String toString() {
        final DecimalFormat format = new DecimalFormat("#0.000");
        return format.format(x_) + "," + format.format(y_) + "," + format.format(getRadius());
    }
}
