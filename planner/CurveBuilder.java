package com.roboracers.topgear.planner;

import com.roboracers.topgear.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class CurveBuilder {

    private static final double DEFAULT_CURVATURE = 0.5;

    public static CubicBezierCurve buildCurve(Vector2d start, Vector2d end, double startTangent, double endTangent) {
        return buildCurve(start, end, startTangent, endTangent, DEFAULT_CURVATURE);
    }

    public static CubicBezierCurve buildCurve(Vector2d start, Vector2d end, double startTangent, double endTangent, double curvature) {
        double distance = start.distanceTo(end);

        Vector2d tangent1 = new Vector2d(Math.cos(startTangent), Math.sin(startTangent));
        Vector2d tangent2 = new Vector2d(Math.cos(endTangent), Math.sin(endTangent));

        Vector2d control1 = start.add(tangent1.multiply(curvature * distance));
        Vector2d control2 = end.add(tangent2.multiply(curvature * distance));

        return new CubicBezierCurve(start, control1, control2, end);
    }

    public static CurveList buildCurveSequence() {
        return new CurveList();
    }

    public static class CurveList {
        private final List<ParametricPath> curves;

        public CurveList() {
            curves = new ArrayList<>();
        }

        public CurveList addCurve(Vector2d start, Vector2d end, double startTangent, double endTangent) {
            curves.add(buildCurve(start, end, startTangent, endTangent));
            return this;
        }

        public CurveList addCurve(Vector2d start, Vector2d end, double startTangent, double endTangent, double curvature) {
            curves.add(buildCurve(start, end, startTangent, endTangent, curvature));
            return this;
        }

        /**
         * Returns the curves added to this sequence so far, in order.
         */
        public List<ParametricPath> getCurves() {
            return Collections.unmodifiableList(curves);
        }

    }
}
