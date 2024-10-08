package com.roboracers.topgear.localization;

import com.roboracers.topgear.geometry.Pose2d;

public interface Localizer {

    /**
     * Current robot pose estimate.
     */
    Pose2d getPoseEstimate();


    void setPoseEstimate(Pose2d pose);

    /**
     * Current robot pose velocity (optional)
     */
    Pose2d getPoseVelocity();

    /**
     * Completes a single localization update.
     */
    void update();
}
