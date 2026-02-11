package com.roboracers.topgear.test;

import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.geometry.Vector2d;

import static com.roboracers.topgear.test.Assert.assertEquals;
import static com.roboracers.topgear.test.Assert.assertTrue;

public class Pose2dTest {

    private static final double EPS = 1e-9;

    public static void runTests() {
        testDefaultConstructor();
        testVecAndHeadingVec();
        testPlusMinus();
        testTimesAndDiv();
        testUnaryMinus();
        testEqualsAndHashCode();
    }

    private static void testDefaultConstructor() {
        Pose2d p = new Pose2d();
        assertEquals("default x", 0.0, p.getX(), EPS);
        assertEquals("default y", 0.0, p.getY(), EPS);
        assertEquals("default heading", 0.0, p.getHeading(), EPS);
    }

    private static void testVecAndHeadingVec() {
        Pose2d p = new Pose2d(3, 4, Math.PI / 2);

        Vector2d vec = p.vec();
        assertEquals("vec x", 3.0, vec.getX(), EPS);
        assertEquals("vec y", 4.0, vec.getY(), EPS);

        Vector2d headingVec = p.headingVec();
        assertEquals("headingVec x at 90deg", 0.0, headingVec.getX(), 1e-9);
        assertEquals("headingVec y at 90deg", 1.0, headingVec.getY(), 1e-9);
    }

    private static void testPlusMinus() {
        Pose2d a = new Pose2d(1, 2, 0.5);
        Pose2d b = new Pose2d(3, -1, 0.25);

        Pose2d sum = a.plus(b);
        assertEquals("plus x", 4.0, sum.getX(), EPS);
        assertEquals("plus y", 1.0, sum.getY(), EPS);
        assertEquals("plus heading", 0.75, sum.getHeading(), EPS);

        Pose2d diff = a.minus(b);
        assertEquals("minus x", -2.0, diff.getX(), EPS);
        assertEquals("minus y", 3.0, diff.getY(), EPS);
        assertEquals("minus heading", 0.25, diff.getHeading(), EPS);
    }

    private static void testTimesAndDiv() {
        Pose2d p = new Pose2d(2, -4, 1.0);

        Pose2d scaled = p.times(2.0);
        assertEquals("times x", 4.0, scaled.getX(), EPS);
        assertEquals("times y", -8.0, scaled.getY(), EPS);
        assertEquals("times heading", 2.0, scaled.getHeading(), EPS);

        Pose2d divided = p.div(2.0);
        assertEquals("div x", 1.0, divided.getX(), EPS);
        assertEquals("div y", -2.0, divided.getY(), EPS);
        assertEquals("div heading", 0.5, divided.getHeading(), EPS);
    }

    private static void testUnaryMinus() {
        Pose2d p = new Pose2d(2, -4, 1.0);
        Pose2d negated = p.unaryMinus();

        assertEquals("unaryMinus x", -2.0, negated.getX(), EPS);
        assertEquals("unaryMinus y", 4.0, negated.getY(), EPS);
        assertEquals("unaryMinus heading", -1.0, negated.getHeading(), EPS);
    }

    private static void testEqualsAndHashCode() {
        Pose2d a = new Pose2d(1, 2, 3);
        Pose2d b = new Pose2d(1, 2, 3);
        Pose2d c = new Pose2d(1, 2, 3.1);

        assertTrue("equal poses should be equal()", a.equals(b));
        assertTrue("equal poses should have equal hashCode()", a.hashCode() == b.hashCode());
        assertTrue("differing poses should not be equal()", !a.equals(c));
    }

    public static void main(String[] args) {
        runTests();
        System.out.println("Pose2dTest: all assertions passed");
    }
}
