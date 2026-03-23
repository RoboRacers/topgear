package com.roboracers.topgear.test;

import com.roboracers.topgear.utils.AngleUtils;

import static com.roboracers.topgear.test.Assert.assertEquals;
import static com.roboracers.topgear.test.Assert.assertTrue;

public class AngleUtilsTest {

    private static final double EPS = 1e-9;
    private static final double PI = Math.PI;
    private static final double TAU = 2 * PI;

    public static void runTests() {
        testNormInRange();
        testNormOfZeroAndTau();
        testNormOfNegativeAngle();
        testNormOfLargeMultiples();
        testNormDeltaWithinRange();
        testNormDeltaWrapsPositiveOverflow();
        testNormDeltaWrapsNegativeOverflow();
        testNormDeltaBoundaryAtPi();
    }

    private static void testNormInRange() {
        for (double angle = -4 * PI; angle <= 4 * PI; angle += 0.37) {
            double normed = AngleUtils.norm(angle);
            assertTrue("norm(" + angle + ") = " + normed + " should be within [0, 2pi)", normed >= 0 && normed < TAU + EPS);
        }
    }

    private static void testNormOfZeroAndTau() {
        assertEquals("norm(0)", 0.0, AngleUtils.norm(0), EPS);
        assertEquals("norm(2pi)", 0.0, AngleUtils.norm(TAU), 1e-6);
    }

    private static void testNormOfNegativeAngle() {
        assertEquals("norm(-pi/2)", 3 * PI / 2, AngleUtils.norm(-PI / 2), 1e-9);
    }

    private static void testNormOfLargeMultiples() {
        assertEquals("norm(10*pi) wraps to 0", 0.0, AngleUtils.norm(10 * PI), 1e-6);
    }

    private static void testNormDeltaWithinRange() {
        for (double angle = -8 * PI; angle <= 8 * PI; angle += 0.53) {
            double normed = AngleUtils.normDelta(angle);
            assertTrue("normDelta(" + angle + ") = " + normed + " should be within [-pi, pi]", normed >= -PI - EPS && normed <= PI + EPS);
        }
    }

    private static void testNormDeltaWrapsPositiveOverflow() {
        assertEquals("normDelta(3pi/2) wraps to -pi/2", -PI / 2, AngleUtils.normDelta(3 * PI / 2), 1e-9);
    }

    private static void testNormDeltaWrapsNegativeOverflow() {
        // -pi/2 is already within [-pi, pi], should pass through unchanged
        assertEquals("normDelta(-pi/2) unchanged", -PI / 2, AngleUtils.normDelta(-PI / 2), 1e-9);
    }

    private static void testNormDeltaBoundaryAtPi() {
        assertEquals("normDelta(pi) stays at pi", PI, AngleUtils.normDelta(PI), 1e-9);
        assertEquals("normDelta(-pi) maps to pi (equivalent angle)", PI, AngleUtils.normDelta(-PI), 1e-9);
    }

    public static void main(String[] args) {
        runTests();
        System.out.println("AngleUtilsTest: all assertions passed");
    }
}
