package com.roboracers.topgear.geometry;

import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2d add(Vector2d v) {
        return new Vector2d(this.x + v.x, this.y + v.y);
    }

    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d multiply(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    public double dot(Vector2d v) {
        return this.x * v.x + this.y * v.y;
    }

    public double length() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2d normalize() {
        double length = length();
        if (length == 0) {
            throw new ArithmeticException("Cannot normalize a zero-length vector");
        }
        return new Vector2d(x / length, y / length);
    }

    public Vector2d scalarMultiply(double scalar) {
        return multiply(scalar);
    }

    public double distanceTo(Vector2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Vector2d rotated(double angle) {
        double newX = x * cos(angle) - y * sin(angle);
        double newY = x * sin(angle) + y * cos(angle);
        return new Vector2d(newX, newY);
    }

    public Vector2d fieldToRobotCentric(double heading) {
        // Apply the inverse rotation matrix to the vector
        double cosTheta = cos(heading);
        double sinTheta = sin(heading);

        double xRobot = cosTheta * this.x + sinTheta * this.y;
        double yRobot = -sinTheta * this.x + cosTheta * this.y;

        // Return the new vector in the robot's frame of reference
        return new Vector2d(xRobot, yRobot);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vector2d)) return false;
        Vector2d other = (Vector2d) o;
        return Double.compare(x, other.x) == 0 && Double.compare(y, other.y) == 0;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(x) * 31 + Double.hashCode(y);
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }

}
