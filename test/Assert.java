package com.roboracers.topgear.test;

/**
 * Minimal assertion helpers for this module's plain-Java tests. There's no build system in
 * this repo (it's a source-only module meant to be dropped into a robot project), so tests
 * here are ordinary classes runnable with javac/java rather than JUnit test cases.
 */
public final class Assert {

    private Assert() {
    }

    public static void assertTrue(String message, boolean condition) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }

    public static void assertEquals(String message, double expected, double actual, double epsilon) {
        if (Double.isNaN(expected) != Double.isNaN(actual) || Math.abs(expected - actual) > epsilon) {
            throw new AssertionError(message + ": expected " + expected + " but was " + actual + " (epsilon=" + epsilon + ")");
        }
    }

    public static void assertEquals(String message, Object expected, Object actual) {
        boolean equal = (expected == null) ? (actual == null) : expected.equals(actual);
        if (!equal) {
            throw new AssertionError(message + ": expected " + expected + " but was " + actual);
        }
    }
}
