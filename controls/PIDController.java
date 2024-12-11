package com.roboracers.topgear.controls;

/**
 * The PIDController class implements a simple Proportional-Integral-Derivative (PID) controller.
 * This controller can be used to regulate the behavior of various systems, such as maintaining a
 * desired speed, position, or other measurable quantity.
 *
 * @version 1.1
 * @since 2024-07-28
 */
public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain
    private double setpoint; // Desired value
    private double previousError; // Previous error value
    private double integral; // Integral term

    // Timestamp (nanoTime) of the last update() call. -1 means "no previous call yet".
    private long lastUpdateNanos = -1;

    /**
     * Constructs a PIDController with the specified gains.
     *
     * @param kp the proportional gain
     * @param ki the integral gain
     * @param kd the derivative gain
     */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = 0.0;
        this.previousError = 0.0;
        this.integral = 0.0;
    }

    public PIDController(PIDCoefficients coefficients) {
        this(coefficients.kp, coefficients.ki, coefficients.kd);
    }

    /**
     * Sets the desired setpoint value for the PID controller.
     *
     * @param setpoint the desired setpoint value
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Resets the controller's internal state (integral term, previous error, and timing).
     * Call this after a period where update() was not called, otherwise the next update()
     * will treat the gap as elapsed time and spike the integral/derivative terms.
     */
    public void reset() {
        this.previousError = 0.0;
        this.integral = 0.0;
        this.lastUpdateNanos = -1;
    }

    /**
     * Updates the PID controller with the current measured value and returns the control output.
     * The integral and derivative terms are computed using the actual elapsed time (dt) since
     * the previous call to update(), not the total time since the controller was constructed.
     * @param currentValue the current measured value
     * @return the control output
     */
    public double update(double currentValue) {
        double error = setpoint - currentValue;

        long now = System.nanoTime();
        if (lastUpdateNanos < 0) {
            // First call since construction/reset: there is no valid dt yet, so we can't
            // integrate or differentiate. Just report the proportional term.
            lastUpdateNanos = now;
            previousError = error;
            return kp * error;
        }

        double dt = (now - lastUpdateNanos) / 1e9;
        lastUpdateNanos = now;

        if (dt <= 0) {
            // Guard against duplicate timestamps / clock weirdness rather than dividing by zero.
            return kp * error;
        }

        integral += error * dt;
        double derivative = (error - previousError) / dt;
        double output = (kp * error) + (ki * integral) + (kd * derivative);
        previousError = error;
        return output;
    }
}
