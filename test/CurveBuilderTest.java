package com.roboracers.topgear.test;

import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.CubicBezierCurve;
import com.roboracers.topgear.planner.CurveBuilder;
import com.roboracers.topgear.planner.ParametricPath;

import java.util.List;

import static com.roboracers.topgear.test.Assert.assertEquals;
import static com.roboracers.topgear.test.Assert.assertTrue;

public class CurveBuilderTest {

    private static final double EPS = 1e-9;

    public static void runTests() {
        testBuiltCurveInterpolatesEndpoints();
        testStartDerivativeMatchesStartTangent();
        testEndDerivativeIsAntiparallelToEndTangent();
        testFourArgOverloadMatchesDefaultCurvature();
        testCurveListAccumulatesCurves();
        testCurveListIsUnmodifiable();
    }

    private static void testBuiltCurveInterpolatesEndpoints() {
        Vector2d start = new Vector2d(0, 0);
        Vector2d end = new Vector2d(10, 0);
        CubicBezierCurve curve = CurveBuilder.buildCurve(start, end, 0, 0);

        assertEquals("start x", start.getX(), curve.getPoint(0).getX(), EPS);
        assertEquals("start y", start.getY(), curve.getPoint(0).getY(), EPS);
        assertEquals("end x", end.getX(), curve.getPoint(1).getX(), EPS);
        assertEquals("end y", end.getY(), curve.getPoint(1).getY(), EPS);
    }

    private static void testStartDerivativeMatchesStartTangent() {
        // A curve starting and ending pointing in the +x direction (tangent angle 0).
        CubicBezierCurve curve = CurveBuilder.buildCurve(new Vector2d(0, 0), new Vector2d(10, 0), 0, 0);

        Vector2d derivativeAtStart = curve.getDerivative(0);
        assertTrue("derivative(0) should point in +x direction", derivativeAtStart.getX() > 0);
        assertEquals("derivative(0) y should be ~0", 0.0, derivativeAtStart.getY(), 1e-9);
    }

    private static void testEndDerivativeIsAntiparallelToEndTangent() {
        // buildCurve's endTangent describes the outgoing-handle direction (bezier-editor
        // convention), so the curve's actual arrival direction at t=1 is the *negative* of it.
        // This has always been this method's behavior; this test pins it down as a regression
        // guard rather than asserting what might seem more "intuitive".
        CubicBezierCurve curve = CurveBuilder.buildCurve(new Vector2d(0, 0), new Vector2d(10, 0), 0, 0);

        Vector2d derivativeAtEnd = curve.getDerivative(1);
        assertTrue("derivative(1) should point in -x direction given endTangent=0", derivativeAtEnd.getX() < 0);
        assertEquals("derivative(1) y should be ~0", 0.0, derivativeAtEnd.getY(), 1e-9);
    }

    private static void testFourArgOverloadMatchesDefaultCurvature() {
        Vector2d start = new Vector2d(1, 2);
        Vector2d end = new Vector2d(8, -3);
        double startTangent = 0.3;
        double endTangent = 1.1;

        CubicBezierCurve viaDefault = CurveBuilder.buildCurve(start, end, startTangent, endTangent);
        CubicBezierCurve viaExplicit = CurveBuilder.buildCurve(start, end, startTangent, endTangent, 0.5);

        for (double t = 0.0; t <= 1.0; t += 0.2) {
            Vector2d a = viaDefault.getPoint(t);
            Vector2d b = viaExplicit.getPoint(t);
            assertEquals("4-arg vs 5-arg(0.5) x at t=" + t, a.getX(), b.getX(), EPS);
            assertEquals("4-arg vs 5-arg(0.5) y at t=" + t, a.getY(), b.getY(), EPS);
        }
    }

    private static void testCurveListAccumulatesCurves() {
        CurveBuilder.CurveList curveList = CurveBuilder.buildCurveSequence()
                .addCurve(new Vector2d(0, 0), new Vector2d(1, 0), 0, 0)
                .addCurve(new Vector2d(1, 0), new Vector2d(2, 0), 0, 0, 0.3);

        List<ParametricPath> curves = curveList.getCurves();
        assertTrue("CurveList should contain the two added curves", curves.size() == 2);
        assertEquals("first curve start x", 0.0, curves.get(0).getPoint(0).getX(), EPS);
        assertEquals("second curve end x", 2.0, curves.get(1).getPoint(1).getX(), EPS);
    }

    private static void testCurveListIsUnmodifiable() {
        CurveBuilder.CurveList curveList = CurveBuilder.buildCurveSequence()
                .addCurve(new Vector2d(0, 0), new Vector2d(1, 0), 0, 0);

        boolean threw = false;
        try {
            curveList.getCurves().add(null);
        } catch (UnsupportedOperationException e) {
            threw = true;
        }
        assertTrue("getCurves() should return an unmodifiable view", threw);
    }

    public static void main(String[] args) {
        runTests();
        System.out.println("CurveBuilderTest: all assertions passed");
    }
}
