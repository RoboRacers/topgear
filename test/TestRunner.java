package com.roboracers.topgear.test;

/**
 * Runs every test suite in this package and prints a pass/fail summary.
 * <p>
 * There's no build system wired up in this repo, so this can be compiled and run directly, e.g.
 * (from the repo root, with commons-math3 on the classpath):
 * <pre>
 *   javac -cp commons-math3.jar -d out $(find . -name "*.java")
 *   java -cp out:commons-math3.jar com.roboracers.topgear.test.TestRunner
 * </pre>
 * Exits with a non-zero status if any suite fails.
 */
public class TestRunner {
    private static int passed = 0;
    private static int failed = 0;

    public static void main(String[] args) {
        run("Vector2dTest", Vector2dTest::runTests);
        run("Pose2dTest", Pose2dTest::runTests);
        run("AngleUtilsTest", AngleUtilsTest::runTests);
        run("CubicBezierCurveTest", CubicBezierCurveTest::runTests);
        run("CurveBuilderTest", CurveBuilderTest::runTests);

        System.out.println();
        System.out.println(passed + " passed, " + failed + " failed");
        if (failed > 0) {
            System.exit(1);
        }
    }

    private static void run(String name, Runnable suite) {
        try {
            suite.run();
            System.out.println("PASS  " + name);
            passed++;
        } catch (AssertionError e) {
            System.out.println("FAIL  " + name + ": " + e.getMessage());
            failed++;
        }
    }
}
