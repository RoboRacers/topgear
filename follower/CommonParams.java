package com.roboracers.topgear.follower;

import com.roboracers.topgear.controls.PIDCoefficients;

public class CommonParams {
    double tangentDistance;
    double centripetalMass;
    double maxSpeed;
    double maxDecel;
    double PIDThreshold;
    double stoppingDistanceThreshold;
    double stoppingPowerThreshold;

    PIDCoefficients xPIDCoeffs;
    PIDCoefficients yPIDCoeffs;
    PIDCoefficients headingPIDCoeffs;

    /**
     * For the theoretical follower, we need to know the tangent distance, centripetal mass, max speed, max deceleration, stopping distance threshold, stopping power threshold, and the PID coefficients for the heading controller.
     * @param tangentDistance
     * @param centripetalMass
     * @param maxSpeed
     * @param maxDecel
     * @param stoppingDistanceThreshold
     * @param stoppingPowerThreshold
     * @param headingPIDCoeffs
     */
    public CommonParams(double tangentDistance, double centripetalMass, double maxSpeed, double maxDecel, double stoppingDistanceThreshold, double stoppingPowerThreshold, PIDCoefficients headingPIDCoeffs) {
        this.tangentDistance = tangentDistance;
        this.centripetalMass = centripetalMass;
        this.maxSpeed = maxSpeed;
        this.maxDecel = maxDecel;
        this.stoppingDistanceThreshold = stoppingDistanceThreshold;
        this.stoppingPowerThreshold = stoppingPowerThreshold;

        this.headingPIDCoeffs = headingPIDCoeffs;
    }

    /**
     * For the centripetal follower, we need to know the tangent distance, centripetal mass, max speed, PID threshold, max deceleration, stopping distance threshold, stopping power threshold, and the PID coefficients for the heading controller.
     * @param tangentDistance
     * @param centripetalMass
     * @param maxSpeed
     * @param PIDThreshold
     * @param stoppingDistanceThreshold
     * @param stoppingPowerThreshold
     * @param headingPIDCoeffs
     */
    public CommonParams(double tangentDistance, double centripetalMass, double maxSpeed, double PIDThreshold, double stoppingDistanceThreshold, double stoppingPowerThreshold, PIDCoefficients xPIDCoeffs, PIDCoefficients yPIDCoeffs, PIDCoefficients headingPIDCoeffs) {
        this.tangentDistance = tangentDistance;
        this.centripetalMass = centripetalMass;
        this.maxSpeed = maxSpeed;
        this.PIDThreshold = PIDThreshold;
        this.maxDecel = maxDecel;
        this.stoppingDistanceThreshold = stoppingDistanceThreshold;
        this.stoppingPowerThreshold = stoppingPowerThreshold;
        this.xPIDCoeffs = xPIDCoeffs;
        this.yPIDCoeffs = yPIDCoeffs;
        this.headingPIDCoeffs = headingPIDCoeffs;
    }

    // combine all the last two constructors into one

}