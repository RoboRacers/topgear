package com.roboracers.topgear.test;

import com.roboracers.topgear.geometry.Vector2d;

import static com.roboracers.topgear.test.Assert.assertEquals;
import static com.roboracers.topgear.test.Assert.assertTrue;

public class Vector2dTest {

    private static final double EPS = 1e-9;

    public static void runTests() {
        testAddSubtract();
        testMultiplyAndScalarMultiplyAgree();
        testDotAndLength();
        testNormalize();
        testNormalizeZeroThrows();
        testDistanceTo();
        testRotated90Degrees();
        testFieldToRobotCentricIsInverseRotation();
        testEqualsAndHashCode();
    }

    private static void testAddSubtract() {
        Vector2d a = new Vector2d(1, 2);
        Vector2d b = new Vector2d(3, -1);

        Vector2d sum = a.add(b);
        assertEquals("add x", 4.0, sum.getX(), EPS);
        assertEquals("add y", 1.0, sum.getY(), EPS);

        Vector2d diff = a.subtract(b);
        assertEquals("subtract x", -2.0, diff.getX(), EPS);
        assertEquals("subtract y", 3.0, diff.getY(), EPS);
    }

    private static void testMultiplyAndScalarMultiplyAgree() {
        Vector2d v = new Vector2d(2, -3);
        Vector2d viaMultiply = v.multiply(2.5);
        Vector2d viaScalarMultiply = v.scalarMultiply(2.5);

        assertEquals("multiply/scalarMultiply x", viaMultiply.getX(), viaScalarMultiply.getX(), EPS);
        assertEquals("multiply/scalarMultiply y", viaMultiply.getY(), viaScalarMultiply.getY(), EPS);
        assertEquals("multiply x", 5.0, viaMultiply.getX(), EPS);
        assertEquals("multiply y", -7.5, viaMultiply.getY(), EPS);
    }

    private static void testDotAndLength() {
        Vector2d a = new Vector2d(3, 4);
        Vector2d b = new Vector2d(1, 0);

        assertEquals("dot", 3.0, a.dot(b), EPS);
        assertEquals("length of (3,4)", 5.0, a.length(), EPS);
    }

    private static void testNormalize() {
        Vector2d v = new Vector2d(3, 4);
        Vector2d n = v.normalize();

        assertEquals("normalized length", 1.0, n.length(), EPS);
        assertEquals("normalized x", 0.6, n.getX(), EPS);
        assertEquals("normalized y", 0.8, n.getY(), EPS);
    }

    private static void testNormalizeZeroThrows() {
        boolean threw = false;
        try {
            new Vector2d(0, 0).normalize();
        } catch (ArithmeticException e) {
            threw = true;
        }
        assertTrue("normalize() of a zero vector should throw ArithmeticException", threw);
    }

    private static void testDistanceTo() {
        Vector2d a = new Vector2d(0, 0);
        Vector2d b = new Vector2d(3, 4);
        assertEquals("distanceTo", 5.0, a.distanceTo(b), EPS);
    }

    private static void testRotated90Degrees() {
        Vector2d v = new Vector2d(1, 0);
        Vector2d rotated = v.rotated(Math.PI / 2);

        assertEquals("rotated 90deg x", 0.0, rotated.getX(), 1e-6);
        assertEquals("rotated 90deg y", 1.0, rotated.getY(), 1e-6);
    }

    private static void testFieldToRobotCentricIsInverseRotation() {
        Vector2d v = new Vector2d(2.5, -1.3);
        double heading = 0.9;

        Vector2d viaFieldToRobot = v.fieldToRobotCentric(heading);
        Vector2d viaInverseRotate = v.rotated(-heading);

        assertEquals("fieldToRobotCentric x matches rotated(-heading)", viaInverseRotate.getX(), viaFieldToRobot.getX(), 1e-9);
        assertEquals("fieldToRobotCentric y matches rotated(-heading)", viaInverseRotate.getY(), viaFieldToRobot.getY(), 1e-9);
    }

    private static void testEqualsAndHashCode() {
        Vector2d a = new Vector2d(1.5, -2.5);
        Vector2d b = new Vector2d(1.5, -2.5);
        Vector2d c = new Vector2d(1.5, 2.5);

        assertTrue("equal vectors should be equal()", a.equals(b));
        assertTrue("equal vectors should have equal hashCode()", a.hashCode() == b.hashCode());
        assertTrue("differing vectors should not be equal()", !a.equals(c));
    }

    public static void main(String[] args) {
        runTests();
        System.out.println("Vector2dTest: all assertions passed");
    }
}
