package com.roboracers.topgear.test;

import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.CubicBezierCurve;

import static com.roboracers.topgear.test.Assert.assertEquals;
import static com.roboracers.topgear.test.Assert.assertTrue;

public class CubicBezierCurveTest {

    private static final double EPS = 1e-9;

    public static void runTests() {
        testEndpointsInterpolateControlPoints();
        testStraightLineHasZeroCurvatureAndInfiniteRadius();
        testStraightLineArcLengthMatchesDistance();
        testCurvedPathDerivativeAtEndpoints();
        testCurvedPathRadiusIsInverseOfCurvature();
    }

    private static void testEndpointsInterpolateControlPoints() {
        Vector2d p0 = new Vector2d(0, 0);
        Vector2d p1 = new Vector2d(1, 2);
        Vector2d p2 = new Vector2d(3, 2);
        Vector2d p3 = new Vector2d(4, 0);
        CubicBezierCurve curve = new CubicBezierCurve(p0, p1, p2, p3);

        Vector2d start = curve.getPoint(0);
        Vector2d end = curve.getPoint(1);

        assertEquals("getPoint(0) x", p0.getX(), start.getX(), EPS);
        assertEquals("getPoint(0) y", p0.getY(), start.getY(), EPS);
        assertEquals("getPoint(1) x", p3.getX(), end.getX(), EPS);
        assertEquals("getPoint(1) y", p3.getY(), end.getY(), EPS);
    }

    private static void testStraightLineHasZeroCurvatureAndInfiniteRadius() {
        // All control points colinear on the x-axis, evenly spaced -> a straight line.
        CubicBezierCurve line = new CubicBezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0), new Vector2d(2, 0), new Vector2d(3, 0));

        for (double t = 0.0; t <= 1.0; t += 0.25) {
            assertEquals("curvature of a straight line at t=" + t, 0.0, line.getCurvature(t), EPS);
            assertTrue("radius of curvature of a straight line at t=" + t + " should be infinite",
                    Double.isInfinite(line.getRadiusOfCurvature(t)));
        }

        Vector2d midpoint = line.getPoint(0.5);
        assertEquals("straight line midpoint x", 1.5, midpoint.getX(), EPS);
        assertEquals("straight line midpoint y", 0.0, midpoint.getY(), EPS);
    }

    private static void testStraightLineArcLengthMatchesDistance() {
        Vector2d start = new Vector2d(0, 0);
        Vector2d end = new Vector2d(3, 0);
        CubicBezierCurve line = new CubicBezierCurve(start, new Vector2d(1, 0), new Vector2d(2, 0), end);

        assertEquals("arc length of a straight line equals endpoint distance",
                start.distanceTo(end), line.getArcLength(), 1e-6);
    }

    private static void testCurvedPathDerivativeAtEndpoints() {
        Vector2d p0 = new Vector2d(0, 0);
        Vector2d p1 = new Vector2d(0, 1);
        Vector2d p2 = new Vector2d(1, 1);
        Vector2d p3 = new Vector2d(1, 0);
        CubicBezierCurve curve = new CubicBezierCurve(p0, p1, p2, p3);

        // For a cubic bezier, derivative(0) == 3*(p1 - p0) and derivative(1) == 3*(p3 - p2).
        Vector2d derivativeAtStart = curve.getDerivative(0);
        assertEquals("derivative(0) x", 0.0, derivativeAtStart.getX(), 1e-9);
        assertEquals("derivative(0) y", 3.0, derivativeAtStart.getY(), 1e-9);

        Vector2d derivativeAtEnd = curve.getDerivative(1);
        assertEquals("derivative(1) x", 0.0, derivativeAtEnd.getX(), 1e-9);
        assertEquals("derivative(1) y", -3.0, derivativeAtEnd.getY(), 1e-9);
    }

    private static void testCurvedPathRadiusIsInverseOfCurvature() {
        CubicBezierCurve curve = new CubicBezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 1), new Vector2d(1, 1), new Vector2d(1, 0));

        double t = 0.5;
        double curvature = curve.getCurvature(t);
        double radius = curve.getRadiusOfCurvature(t);

        assertTrue("this S-curve should have nonzero curvature at t=0.5", curvature > EPS);
        assertEquals("radius of curvature should be 1/curvature", 1.0 / curvature, radius, 1e-6);
    }

    public static void main(String[] args) {
        runTests();
        System.out.println("CubicBezierCurveTest: all assertions passed");
    }
}
