package com.roboracers.topgear.follower;


import com.roboracers.topgear.controls.PIDCoefficients;
import com.roboracers.topgear.controls.PIDController;
import com.roboracers.topgear.geometry.PointProjection;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.ParametricPath;


/**
 * The GuidedVectorFieldFollower class implements a robot controller that follows a guided vector field (GVF).
 * This class provides methods to set the desired vector field and update the robot's position
 * based on the GVF.
 * <p>
 * GVF is a technique used in robotics for path following where the robot follows a vector field
 * that guides it along a desired path. This class handles the computation of control signals
 * to adjust the robot's movement to stay on the path defined by the GVF.
 * </p>
 */
// gvf without pid ending
public class EndScalingGuidedVectorFieldFollower extends AbstractGuidedVectorFieldFollower {

    /**
     * The distance between the closest point and the tangent point.
     */
    private double tangentDistance;
    /**
     * Used to compute the centripetal force correction.
     */
    private double centripetalMass;
    /**
     * Max speed of the robot while following the path.
     */
    private double maxSpeed;
    /**
     * The maximum deceleration of the robot. Controls end point velocity scaling.
     */
    private double maxDecel;

    /**
     * Heading PID controller.
     */
    PIDController headingPID;

    public EndScalingGuidedVectorFieldFollower(Params params) {
        this.tangentDistance = params.tangentDistance;
        this.maxSpeed = params.maxSpeed;
        this.maxDecel = params.maxDecel;
        this.stoppingDistanceThreshold = params.stoppingDistanceThreshold;
        this.stoppingPowerThreshold = params.stoppingPowerThreshold;

        headingPID = new PIDController(params.headingPIDCoeffs);
    }

    /**
     * Parameters for the GVF follower.
     */
    public static class Params {
        double tangentDistance;
        double centripetalMass;
        double maxSpeed;
        double maxDecel;
        double stoppingDistanceThreshold;
        double stoppingPowerThreshold;

        PIDCoefficients xPIDCoeffs;
        PIDCoefficients yPIDCoeffs;
        PIDCoefficients headingPIDCoeffs;

        public Params(double tangentDistance, double centripetalMass, double maxSpeed, double maxDecel, double stoppingDistanceThreshold, double stoppingPowerThreshold, PIDCoefficients headingPIDCoeffs) {
            this.tangentDistance = tangentDistance;
            this.centripetalMass = centripetalMass;
            this.maxSpeed = maxSpeed;
            this.maxDecel = maxDecel;
            this.stoppingDistanceThreshold = stoppingDistanceThreshold;
            this.stoppingPowerThreshold = stoppingPowerThreshold;
            this.headingPIDCoeffs = headingPIDCoeffs;
        }
    }

    /**
     * Feeds the drive powers to the drivetrain based on the direction of the
     * vector gradient field at the current point. Only provides x and y translation,
     * no heading in this implementation.
     * @param currentPosition robot current position
     * @return Drive power
     */
    @Override
    public Pose2d getDriveVelocity(Pose2d currentPosition, Pose2d currentVelocity) {
        // If no path has been set, do not return anything
        if (this.parametricPath == null)
            return null;

        Vector2d endpoint = parametricPath.getPoint(1);
        double distanceToEnd = currentPosition.vec().distanceTo(endpoint);

        Vector2d currentPoint = currentPosition.vec();

        // Find the closest point on the path from the robot and get its t-value
        double closestTValue = PointProjection.projectionBinarySearch(parametricPath, currentPoint, 10);

        // Calculate the tangent point (point that the robot goes towards
        Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

        // Get the vector pointing from the robot to the tangent point
        Vector2d driveVector = tangentPoint.subtract(currentPoint);

        // Compute centripetal force correction to make sure the robot stays on the path even in high curvature sections
        Vector2d centripetalForceCorrection = computeCentripetalForceCorrection(closestTValue, currentVelocity);

        // Add both main drive vector
        Vector2d normalizedVector = (driveVector.add(centripetalForceCorrection)).normalize();

        // Slow down the robot based on the distance from the end
        double endScaling = Math.min(Math.sqrt(2*distanceToEnd*maxDecel),1);

        // Scale the speed by the max speed
        Vector2d velocityVector = normalizedVector.scalarMultiply(maxSpeed).scalarMultiply(endScaling);

        // Rotate the vector from a global frame of reference to the robot's frame of reference
        Vector2d robotFrame = velocityVector.rotated(-currentPosition.getHeading());

        // Calculate the heading target
        double headingTarget = Math.atan2(robotFrame.getY(), robotFrame.getX());
        headingPID.setSetpoint(headingTarget);

        Pose2d drivePower = new Pose2d(robotFrame, headingPID.update(currentPosition.getHeading()));

        // Debugging values
        usingPID = false;
        currentClosestTValue = closestTValue;
        currentDistanceToEnd = distanceToEnd;
        currentClosestPoint = parametricPath.getPoint(closestTValue);
        currentTangentPoint = tangentPoint;
        currentDrivePower = drivePower;
        currentHeadingTarget = headingTarget;

        // Return the new vector in the robot's frame of reference
        return drivePower;

    }

    // Function to calculate the centripetal force correction at a point on the curve
    public Vector2d computeCentripetalForceCorrection(double t, Pose2d velocity) {
        double radiusOfCurvature = parametricPath.getRadiusOfCurvature(t);

        // If the radius of curvature is infinite (straight path), no correction is needed
        if (radiusOfCurvature == Double.POSITIVE_INFINITY) {
            return new Vector2d(0, 0); // No correction needed for straight paths
        }

        // Compute the centripetal force
        Vector2d tangentUnitVector = parametricPath.getDerivative(t).normalize();
        double tangentVelocity = velocity.vec().dot(tangentUnitVector);
        double centripetalForceMagnitude = (centripetalMass * tangentVelocity * tangentVelocity) / radiusOfCurvature;

        // Get the unit vector normal to the path (pointing towards the center of curvature)
        Vector2d tangent = parametricPath.getDerivative(t).normalize(); // Tangent vector
        Vector2d normal = new Vector2d(-tangent.getY(), tangent.getX()); // Perpendicular to tangent

        // The direction of the normal force depends on the curve's orientation
        if (parametricPath.getCurvature(t) < 0) {
            normal = normal.multiply(-1); // Flip the normal direction if curvature is negative
        }

        // The centripetal force correction is in the direction of the normal
        return normal.multiply(centripetalForceMagnitude);
    }

    /**
     * DebugPacket class to store the current state of the follower.
     */
    public class DebugPacket {
        public boolean usingPID;
        public double currentClosestTValue;
        public double currentDistanceToEnd;
        public Vector2d currentClosestPoint;
        public Vector2d currentTangentPoint;
        public Pose2d currentDrivePower;
        public double currentHeadingTarget;

        public DebugPacket(EndScalingGuidedVectorFieldFollower follower) {
            this.usingPID = follower.usingPID;
            this.currentClosestTValue = follower.currentClosestTValue;
            this.currentDistanceToEnd = follower.currentDistanceToEnd;
            this.currentClosestPoint = follower.currentClosestPoint;
            this.currentTangentPoint = follower.currentTangentPoint;
            this.currentDrivePower = follower.currentDrivePower;
            this.currentHeadingTarget = follower.currentHeadingTarget;
        }


        @Override
        public String toString() {
            return "DebugPacket for GVF Follower {" +
                    "usingPID=" + usingPID +
                    ", currentClosestTValue=" + currentClosestTValue +
                    ", currentDistanceToEnd=" + currentDistanceToEnd +
                    ", currentClosestPoint=" + currentClosestPoint +
                    ", currentTangentPoint=" + currentTangentPoint +
                    ", currentDrivePower=" + currentDrivePower +
                    ", currentHeadingTarget=" + currentHeadingTarget +
                    '}';
        }
    }

    /**
     * Returns useful information about the current state of the follower.
     * @return DebugPacket
     */
    public DebugPacket getDebugPacket() {
        return new DebugPacket(this);
    }
}

