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
public class GuidedVectorFieldFollower implements Follower {

    /**
     * Current parametrically defined path that is being follower.
     */
    private ParametricPath parametricPath;
    /**
     * The distance between the closest point and the tangent point.
     */
    private double tangentDistance;
    /**
     * Max speed of the robot while following the path.
     */
    private double maxSpeed;
    /**
     * Threshold for the end PID to kick in, measured in inches.
     */
    private double PIDThreshold;
    /**
     * The distance threshold for the end of the path, measured in inches.
     */
    private double stoppingDistanceThreshold;
    /**
     * The minimum power for the end of the path, measured between 0 and 1.
     */
    private double stoppingPowerThreshold;

    /**
     * X PID controller.
     */
    PIDController xPID;
    /**
     * Y PID controller.
     */
    PIDController yPID;
    /**
     * Heading PID controller.
     */
    PIDController headingPID;

    public GuidedVectorFieldFollower(double tangentDistance) {
        if (tangentDistance <= 0)
            this.tangentDistance = 0.1;
        else
            this.tangentDistance = tangentDistance;


        xPID = new PIDController(1,0,0);
        yPID = new PIDController(1,0,0);
        headingPID = new PIDController(1,0,0);
    }


    public GuidedVectorFieldFollower(double tangentDistance, PIDCoefficients xPIDCoeffs, PIDCoefficients yPIDCoeffs, PIDCoefficients headingPIDCoeffs, double maxSpeed) {
        if (tangentDistance <= 0)
            this.tangentDistance = 0.1;
        else
            this.tangentDistance = tangentDistance;


        xPID = new PIDController(xPIDCoeffs);
        yPID = new PIDController(yPIDCoeffs);
        headingPID = new PIDController(headingPIDCoeffs);
        this.maxSpeed = maxSpeed;
    }

    public GuidedVectorFieldFollower(Params params) {
        this.tangentDistance = params.tangentDistance;
        this.maxSpeed = params.maxSpeed;
        this.PIDThreshold = params.PIDThreshold;
        this.stoppingDistanceThreshold = params.stoppingDistanceThreshold;
        this.stoppingPowerThreshold = params.stoppingPowerThreshold;

        xPID = new PIDController(params.xPIDCoeffs);
        yPID = new PIDController(params.yPIDCoeffs);
        headingPID = new PIDController(params.headingPIDCoeffs);
    }

    /**
     * Parameters for the GVF follower.
     */
    public static class Params {
        double tangentDistance;
        double maxSpeed;
        double PIDThreshold;
        double stoppingDistanceThreshold;
        double stoppingPowerThreshold;

        PIDCoefficients xPIDCoeffs;
        PIDCoefficients yPIDCoeffs;
        PIDCoefficients headingPIDCoeffs;

        public Params(double tangentDistance, double maxSpeed, double PIDThreshold, double stoppingDistanceThreshold, double stoppingPowerThreshold, PIDCoefficients xPIDCoeffs, PIDCoefficients yPIDCoeffs, PIDCoefficients headingPIDCoeffs) {
            this.tangentDistance = tangentDistance;
            this.maxSpeed = maxSpeed;
            this.PIDThreshold = PIDThreshold;
            this.stoppingDistanceThreshold = stoppingDistanceThreshold;
            this.stoppingPowerThreshold = stoppingPowerThreshold;
            this.xPIDCoeffs = xPIDCoeffs;
            this.yPIDCoeffs = yPIDCoeffs;
            this.headingPIDCoeffs = headingPIDCoeffs;
        }
    }

    /**
     * Set the current path to be followed.
     * @param parametricPath
     */
    @Override
    public void setPath(ParametricPath parametricPath) {
        this.parametricPath = parametricPath;
    }

    @Override
    public ParametricPath getPath() {
        return this.parametricPath;
    }

    public ParametricPath getParametricPath() {
        return parametricPath;
    }
    /**
     * Feeds the drive powers to the drivetrain based on the direction of the
     * vector gradient field at the current point. Only provides x and y translation,
     * no heading in this implementation.
     * @param currentPosition robot current position
     * @return Drive power
     */
    public Pose2d getDriveVelocity0(Pose2d currentPosition) {

        // If no path has been set, do not return anything
        if (this.parametricPath == null)
            return null;

        Vector2d endpoint = parametricPath.getPoint(1);
        double distanceToEnd = currentPosition.vec().distanceTo(endpoint);

        // Use the ending PID while within threshold.
        if (Math.abs(distanceToEnd) < PIDThreshold) {
            Vector2d endDerivative = parametricPath.getDerivative(0.99);

            double headingTarget = Math.atan2(endDerivative.getY(), endDerivative.getX());

            // Debugging values
            usingPID = true;
            currentClosestTValue = 0;
            currentDistanceToEnd = distanceToEnd;
            currentClosestPoint = new Vector2d(0,0);
            currentTangentPoint = new Vector2d(0,0);
            currentDrivePower = new Pose2d();
            currentHeadingTarget = headingTarget;

            return PIDToPoint( new Pose2d(endpoint, headingTarget), currentPosition);

        } else {
            usingPID = false;
            Vector2d currentPoint = currentPosition.vec();

            // Find the closest point on the path from the robot and get its t-value
            double closestTValue = PointProjection.projectionBinarySearch(parametricPath, currentPoint, 10);

            // Calculate the tangent point (point that the robot goes towards
            Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                    parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

            // Get the vector pointing from the robot to the tangent point
            Vector2d connectingVector = tangentPoint.subtract(currentPoint);
            Vector2d normalizedVector = connectingVector.normalize();

            // Scale the speed by the max speed
            Vector2d velocityVector = normalizedVector.scalarMultiply(maxSpeed);

            // Rotate the vector to the robot's frame of reference
            Vector2d robotFrame = velocityVector.rotated(-currentPosition.getHeading());

            // Calculate the heading target
            double headingTarget = Math.atan2(robotFrame.getY(), robotFrame.getX());
            headingPID.setSetpoint(headingTarget);

            Pose2d drivePower = new Pose2d(robotFrame, headingPID.update(currentPosition.getHeading()));

            parametricPath.getRadiusOfCurvature(closestTValue);

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

        // Use the ending PID while within threshold.
        if (Math.abs(distanceToEnd) < PIDThreshold) {
            Vector2d endDerivative = parametricPath.getDerivative(0.99);

            double headingTarget = Math.atan2(endDerivative.getY(), endDerivative.getX());

            // Debugging values
            usingPID = true;
            currentClosestTValue = 0;
            currentDistanceToEnd = distanceToEnd;
            currentClosestPoint = new Vector2d(0,0);
            currentTangentPoint = new Vector2d(0,0);
            currentDrivePower = new Pose2d();
            currentHeadingTarget = headingTarget;

            return PIDToPoint( new Pose2d(endpoint, headingTarget), currentPosition);

        } else {
            usingPID = false;
            Vector2d currentPoint = currentPosition.vec();

            // Find the closest point on the path from the robot and get its t-value
            double closestTValue = PointProjection.projectionBinarySearch(parametricPath, currentPoint, 10);

            // Calculate the tangent point (point that the robot goes towards
            Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                    parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

            // Get the vector pointing from the robot to the tangent point
            Vector2d connectingVector = tangentPoint.subtract(currentPoint);
            Vector2d centripetalForceCorrection = computeCentripetalForceCorrection(closestTValue, currentVelocity);

            Vector2d normalizedVector = (connectingVector.add(centripetalForceCorrection)).normalize();

            // Scale the speed by the max speed
            Vector2d velocityVector = normalizedVector.scalarMultiply(maxSpeed);

            // Rotate the vector to the robot's frame of reference
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
    }

    /**
     * Check if the robot has reached the end of the path.
     * @param currentPosition robot current position
     * @return true if the robot has reached the end of the path, false otherwise
     */
    @Override
    public Boolean isComplete(Pose2d currentPosition) {
        Vector2d endpoint = parametricPath.getPoint(1);
        double delta =  currentPosition.vec().distanceTo(endpoint);
        if (currentDrivePower == null) currentDrivePower = new Pose2d(0,0,0);
        double power = currentDrivePower.vec().length();
        return delta < stoppingDistanceThreshold
                && power < stoppingPowerThreshold;
    }

    /**
     * PID to point implementation to bring the robot to a stop,
     * or to hold position at the end of the path.
     * @param target
     * @param currentPose
     * @return
     */
    private Pose2d PIDToPoint(Pose2d target, Pose2d currentPose) {

        xPID.setSetpoint(target.getX());
        yPID.setSetpoint(target.getY());
        headingPID.setSetpoint(target.getHeading());

        Vector2d translationPowers = new Vector2d(
                xPID.update(currentPose.getX()),
                yPID.update(currentPose.getY())
        ).rotated(-currentPose.getHeading());

        return new Pose2d(
                translationPowers,
                headingPID.update(currentPose.getHeading())
        );
    }

    // Function to calculate the centripetal force correction at a point on the curve
    public Vector2d computeCentripetalForceCorrection(double t, Pose2d velocity) {
        double radiusOfCurvature = parametricPath.getRadiusOfCurvature(t);

        // If the radius of curvature is infinite (straight path), no correction is needed
        if (radiusOfCurvature == Double.POSITIVE_INFINITY) {
            return new Vector2d(0, 0); // No correction needed for straight paths
        }

        // Compute the centripetal force
        double mass = .07; // .85
        Vector2d tangentUnitVector = parametricPath.getDerivative(t).normalize();
        double tangentVelocity = velocity.vec().dot(tangentUnitVector);
        double centripetalForceMagnitude = (mass * tangentVelocity * tangentVelocity) / radiusOfCurvature;

        // Get the unit vector normal to the path (pointing towards the center of curvature)
        Vector2d tangent = parametricPath.getDerivative(t).normalize(); // Tangent vector
        Vector2d normal = new Vector2d(-tangent.getY(), tangent.getX()); // Perpendicular to tangent

        // The direction of the normal force depends on the curve's orientation
        if (parametricPath.getCurvature(t) < 0) {
            normal = normal.multiply(-1); // Flip the normal direction if curvature is negative
        }

        // The centripetal force correction is in the direction of the normal
        Vector2d centripetalForce = normal.multiply(centripetalForceMagnitude);

        return centripetalForce;
    }


    /*
     * Status variables.
     */
    protected boolean usingPID = false;
    protected double currentClosestTValue;
    protected double currentDistanceToEnd;
    protected Vector2d currentClosestPoint;
    protected Vector2d currentTangentPoint;
    protected Pose2d currentDrivePower;
    protected double currentHeadingTarget;

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


        public DebugPacket(GuidedVectorFieldFollower follower) {
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

