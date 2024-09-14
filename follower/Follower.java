package com.roboracers.topgear.follower;

import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.planner.ParametricPath;

public interface Follower {

    void setPath(ParametricPath parametricPath);
    ParametricPath getPath();

    Pose2d getDriveVelocity(Pose2d currentPosition, Pose2d currentVelocity);

    Boolean isComplete(Pose2d currentPosition);


}
