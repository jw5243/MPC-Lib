package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public class Obstacle {
    private static final double ROBOT_RADIUS = Math.sqrt(2d) * 9d * 0.0254d; //in
    private double lengthScale; //m
    private double obstacleRadius; //m

    private Translation2d location;
    private double costFactor;

    /**
     *
     * @param x
     * @param y
     * @param radius inches
     * @param costFactor
     */
    public Obstacle(double x, double y, double radius, double costFactor) {
        this(new Translation2d(x, y), radius, costFactor);
    }

    /**
     *
     * @param location
     * @param radius inches
     * @param costFactor
     */
    public Obstacle(Translation2d location, double radius, double costFactor) {
        setLocation(location);
        setObstacleRadius(radius * 0.0254d);
        setCostFactor(costFactor);
        setLengthScale((getRobotRadius() + getObstacleRadius()) / 1.7d);
    }

    public double distance(Translation2d other) {
        return getLocation().distance(other) * 0.0254d;
    }

    public double distance(SimpleMatrix state) {
        return distance(new Translation2d(state.get(0) / 0.0254d, state.get(2) / 0.0254d));
    }

    public boolean hittingObstacle(Translation2d other) {
        return (distance(other) - getRobotRadius() * 0.9d) / 0.0254d < 0d;
    }

    public SimpleMatrix stateRepresentation() {
        return new SimpleMatrix(6, 1, true, new double[] {
                getLocation().x() * 0.0254d, 0d, getLocation().y() * 0.0254d, 0d, 0d, 0d
        });
    }

    /**
     *
     *
     * @param expectedState Expected state (via simulation) as part of planned trajectory.
     * @return
     */
    public SimpleMatrix getLinearCost(SimpleMatrix expectedState) {
        SimpleMatrix stateDisplacement = expectedState.minus(stateRepresentation());
        Translation2d displacement = new Translation2d(stateDisplacement.get(0), stateDisplacement.get(2));
        return new SimpleMatrix(6, 1, true, new double[] {
                -2d * displacement.x(), 0, -2d * displacement.y(), 0, 0, 0
        }).scale(getCostFactor() * Math.exp(-displacement.norm2() / Math.pow(getLengthScale(), 2)));
    }

    public SimpleMatrix getQuadraticCost(SimpleMatrix expectedState) {
        SimpleMatrix stateDisplacement = expectedState.minus(stateRepresentation());
        Translation2d displacement = new Translation2d(stateDisplacement.get(0), stateDisplacement.get(2));

        SimpleMatrix Q = new SimpleMatrix(2, 2, true, new double[] {
                -2d * getLengthScale() * getLengthScale() + 4d * displacement.x() * displacement.x(), 4d * displacement.x() * displacement.y(),
                4d * displacement.x() * displacement.y(), -2d * getLengthScale() * getLengthScale() + 4d * displacement.y() * displacement.y()
        }).scale(Math.exp(-displacement.norm2() / Math.pow(getLengthScale(), 2)) / Math.pow(getLengthScale(), 4));

        SimpleEVD<SimpleMatrix> eigenDecomposition = Q.eig();
        if(eigenDecomposition.getNumberOfEigenvalues() == 2) {
            try {
                SimpleMatrix P = new SimpleMatrix(2, 2, true, new double[] {
                        eigenDecomposition.getEigenVector(0).get(0), eigenDecomposition.getEigenVector(1).get(0),
                        eigenDecomposition.getEigenVector(0).get(1), eigenDecomposition.getEigenVector(1).get(1)
                });

                double eigenvalue1 = eigenDecomposition.getEigenvalue(0).getReal();
                double eigenvalue2 = eigenDecomposition.getEigenvalue(1).getReal();
                eigenvalue1 = eigenvalue1 < 0d ? 0d : eigenvalue1;
                eigenvalue2 = eigenvalue2 < 0d ? 0d : eigenvalue2;

                SimpleMatrix D = new SimpleMatrix(2, 2, true, new double[] {
                        eigenvalue1, 0,
                        0, eigenvalue2
                });

                Q = P.mult(D).mult(P.transpose());
            } catch(NullPointerException e) {
                return new SimpleMatrix(6, 6, true, new double[] {
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0
                });
            }
        }

        return new SimpleMatrix(6, 6, true, new double[] {
                Q.get(0, 0), 0, Q.get(0, 1), 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                Q.get(1, 0), 0, Q.get(1, 1), 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        }).scale(getCostFactor());
    }

    public Translation2d getLocation() {
        return location;
    }

    public void setLocation(Translation2d location) {
        this.location = location;
    }

    public double getCostFactor() {
        return costFactor;
    }

    public void setCostFactor(double costFactor) {
        this.costFactor = costFactor;
    }

    public double getLengthScale() {
        return lengthScale;
    }

    public void setLengthScale(double lengthScale) {
        this.lengthScale = lengthScale;
    }

    public static double getRobotRadius() {
        return ROBOT_RADIUS;
    }

    public double getObstacleRadius() {
        return obstacleRadius;
    }

    public void setObstacleRadius(double obstacleRadius) {
        this.obstacleRadius = obstacleRadius;
    }
}
