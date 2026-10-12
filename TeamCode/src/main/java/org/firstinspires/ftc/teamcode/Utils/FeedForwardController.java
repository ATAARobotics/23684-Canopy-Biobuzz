package org.firstinspires.ftc.teamcode.Utils;

/**
 * A simple feedforward controller that calculates motor power based on velocity and acceleration.
 * Uses the formula: output = kS * signum(velocity) + kV * velocity + kA * acceleration
 * <p>
 * Where:
 * - kS (static friction): Constant output to overcome static friction
 * - kV (velocity): Proportional gain for velocity
 * - kA (acceleration): Proportional gain for acceleration
 */
public class FeedForwardController {
    private double kS = 0;
    private double kV = 0;
    private double kA = 0;

    /**
     * Create a FeedForwardController with specified gains.
     *
     * @param kS Static friction gain. Constant output needed to overcome static friction.
     * @param kV Velocity gain. Output proportional to velocity.
     * @param kA Acceleration gain. Output proportional to acceleration.
     */
    public FeedForwardController(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Set the static friction gain (kS).
     *
     * @param kS The static friction gain value
     */
    public void setKS(double kS) {
        this.kS = kS;
    }

    /**
     * Set the velocity gain (kV).
     *
     * @param kV The velocity gain value
     */
    public void setKV(double kV) {
        this.kV = kV;
    }

    /**
     * Set the acceleration gain (kA).
     *
     * @param kA The acceleration gain value
     */
    public void setKA(double kA) {
        this.kA = kA;
    }

    /**
     * Set all feedforward gains at once.
     *
     * @param kS Static friction gain
     * @param kV Velocity gain
     * @param kA Acceleration gain
     */
    public void setGains(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Calculate the feedforward output.
     *
     * @param velocity     The current velocity
     * @param acceleration The current acceleration
     * @return The calculated feedforward output
     */
    public double calculate(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }
}