package com.roboracers.topgear.follower;

import com.roboracers.topgear.controls.PIDController;
import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.geometry.Vector2d;
import com.roboracers.topgear.planner.ParametricPath;

/**
 * Base class for the guided-vector-field (GVF) follower variants in this package.
 * <p>
 * All of the GVF followers share the same path handle, completion check, and status/debug
 * bookkeeping; only the drive-vector computation in getDriveVelocity() differs between them.
 * This class holds the shared parts so each concrete follower only needs to implement its own
 * control law.
 */
public abstract class AbstractGuidedVectorFieldFollower implements Follower {

    /**
     * Current parametrically defined path that is being followed.
     */
    protected ParametricPath parametricPath;
    /**
     * The distance threshold for the end of the path, measured in inches.
     */
    protected double stoppingDistanceThreshold;
    /**
     * The minimum power for the end of the path, measured between 0 and 1.
     */
    protected double stoppingPowerThreshold;

    /*
     * Status variables, exposed via getDebugPacket() by each subclass.
     */
    protected boolean usingPID = false;
    protected double currentClosestTValue;
    protected double currentDistanceToEnd;
    protected Vector2d currentClosestPoint;
    protected Vector2d currentTangentPoint;
    protected Pose2d currentDrivePower;
    protected double currentHeadingTarget;

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
        return parametricPath;
    }

    /**
     * Check if the robot has reached the end of the path.
     * @param currentPosition robot current position
     * @return true if the robot has reached the end of the path, false otherwise
     */
    @Override
    public Boolean isComplete(Pose2d currentPosition) {
        Vector2d endpoint = parametricPath.getPoint(1);
        double delta = currentPosition.vec().distanceTo(endpoint);
        if (currentDrivePower == null) currentDrivePower = new Pose2d(0, 0, 0);
        double power = currentDrivePower.vec().length();
        return delta < stoppingDistanceThreshold
                && power < stoppingPowerThreshold;
    }

    /**
     * PID to point implementation to bring the robot to a stop, or to hold position, using
     * independent x/y/heading PID controllers. Shared by the follower variants that switch to
     * a hold-position phase once within range of the path's endpoint.
     * @param xPID x-axis PID controller
     * @param yPID y-axis PID controller
     * @param headingPID heading PID controller
     * @param target target pose to hold
     * @param currentPose robot's current pose
     * @return drive power, in the robot's frame of reference
     */
    protected static Pose2d pidToPoint(PIDController xPID, PIDController yPID, PIDController headingPID,
                                        Pose2d target, Pose2d currentPose) {
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
}
