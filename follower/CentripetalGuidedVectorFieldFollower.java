package com.roboracers.topgear.follower;


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
public class CentripetalGuidedVectorFieldFollower extends AbstractGuidedVectorFieldFollower {

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
     * Threshold for the end PID to kick in, measured in inches.
     */
    private double PIDThreshold;

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

    public CentripetalGuidedVectorFieldFollower(CommonParams params) {
        this.tangentDistance = params.tangentDistance;
        this.centripetalMass = params.centripetalMass;
        this.maxSpeed = params.maxSpeed;
        this.PIDThreshold = params.PIDThreshold;
        this.stoppingDistanceThreshold = params.stoppingDistanceThreshold;
        this.stoppingPowerThreshold = params.stoppingPowerThreshold;

        xPID = new PIDController(params.xPIDCoeffs);
        yPID = new PIDController(params.yPIDCoeffs);
        headingPID = new PIDController(params.headingPIDCoeffs);
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

            return pidToPoint(xPID, yPID, headingPID, new Pose2d(endpoint, headingTarget), currentPosition);

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
            Vector2d centripetalForceCorrection = computeCentripetalForceCorrection(parametricPath, closestTValue, currentVelocity, centripetalMass);

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


        public DebugPacket(CentripetalGuidedVectorFieldFollower follower) {
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

