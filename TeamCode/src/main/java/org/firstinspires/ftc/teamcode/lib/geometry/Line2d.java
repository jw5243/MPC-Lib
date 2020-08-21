package org.firstinspires.ftc.teamcode.lib.geometry;

public class Line2d {
    private Translation2d startPoint;
    private Translation2d endPoint;

    public Line2d() {
        this(0d, 0d, 0d, 0d);
    }

    public Line2d(Translation2d startPoint, Translation2d endPoint) {
        setStartPoint(startPoint);
        setEndPoint(endPoint);
    }

    public Line2d(double startX, double startY, double endX, double endY) {
        this(new Translation2d(startX, startY), new Translation2d(endX, endY));
    }

    public Line2d(Line2d line2d) {
        this(line2d.getStartPoint(), line2d.getEndPoint());
    }

    @Override
    public String toString() {
        return getStartPoint() + "," + getEndPoint();
    }

    public Translation2d getStartPoint() {
        return startPoint;
    }

    public void setStartPoint(Translation2d startPoint) {
        this.startPoint = startPoint;
    }

    public Translation2d getEndPoint() {
        return endPoint;
    }

    public void setEndPoint(Translation2d endPoint) {
        this.endPoint = endPoint;
    }
}
