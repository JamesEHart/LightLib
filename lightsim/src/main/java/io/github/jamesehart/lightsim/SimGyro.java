package io.github.jamesehart.lightsim;

/** A gyro that LightSim can drive from the simulated drivetrain, e.g. {@code SimPigeon2}. */
public interface SimGyro {
    /**
     * Writes the simulated heading into the gyro.
     *
     * @param yawDegrees continuous (unwrapped) yaw, counter-clockwise positive
     * @param yawRateDegreesPerSecond yaw rate, counter-clockwise positive
     */
    void setSimulatedYaw(double yawDegrees, double yawRateDegreesPerSecond);
}
