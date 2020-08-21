package com.horse.mpclib.lib.geometry;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class Rectangle {
    private static final double DX = 0.5d;

    private Translation2d initialPoint;
    private Translation2d finalPoint;

    private List<Translation2d> discreteRectangle;

    public Rectangle(Translation2d initialPoint, Translation2d finalPoint) {
        this.initialPoint = initialPoint;
        this.finalPoint = finalPoint;
        getRectangleBorderDiscrete(true);
    }

    public Rectangle(double x1, double y1, double x2, double y2) {
        this(new Translation2d(x1, y1), new Translation2d(x2, y2));
    }

    public Translation2d getInitialPoint() {
        return getInitialPoint();
    }

    public Translation2d getFinalPoint() {
        return getFinalPoint();
    }

    public List<Translation2d> getRectangleBorderDiscrete(boolean update) {
        if(update) {
            discreteRectangle = new ArrayList<>();
            double width = finalPoint.x() - initialPoint.x();
            double length = finalPoint.y() - initialPoint.y();

            width = width >= 0 ? width : -width;
            length = length >= 0 ? length : -length;

            int horizontalPoints = (int)(width / DX);
            int verticalPoints = (int)(length / DX);

            for(int i = 0; i < horizontalPoints; i++) {
                discreteRectangle.add(initialPoint.translateBy(new Translation2d(i * DX, 0d)));
                discreteRectangle.add(finalPoint.translateBy(new Translation2d(-i * DX, 0d)));
            }

            for(int j = 0; j < verticalPoints; j++) {
                discreteRectangle.add(initialPoint.translateBy(new Translation2d(0d, j * DX)));
                discreteRectangle.add(finalPoint.translateBy(new Translation2d(0d, -j * DX)));
            }
        }

        return discreteRectangle;
    }

    public List<SimpleMatrix> minkowskiSum(SimpleMatrix matrix) {
        List<SimpleMatrix> sum = new ArrayList<>();
        discreteRectangle.forEach(rectangle -> sum.add(new SimpleMatrix(2, 1, false, new double[] {rectangle.x(), rectangle.y()}).plus(matrix)));

        return sum;
    }

    public static void main(String... args) {
        Rectangle rectangle = new Rectangle(1, 1, 2, 2);
        SimpleMatrix matrix = new SimpleMatrix(2, 1, false, new double[] {1, 1});

        rectangle.minkowskiSum(matrix).forEach(System.out::println);
    }
}
