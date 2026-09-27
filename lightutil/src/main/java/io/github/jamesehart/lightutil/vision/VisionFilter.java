package io.github.jamesehart.lightutil.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Decides whether to trust an AprilTag pose estimate, and how much. Works with any camera library:
 * pass in the numbers from PhotonVision or Limelight and feed the result to your pose estimator.
 *
 * <p>Estimates are rejected if they're off the field, floating or sunk, too ambiguous (single tag),
 * or from tags too far away. Accepted ones get standard deviations that grow with distance and
 * shrink with more tags, so the pose estimator trusts close multi-tag views most.
 *
 * <pre>{@code
 * VisionFilter filter = new VisionFilter();
 * // for each camera result:
 * filter.process(estimate.estimatedPose, estimate.timestampSeconds,
 *         estimate.targetsUsed.size(), avgDistance, ambiguity)
 *     .ifPresent(m -> poseEstimator.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs()));
 * }</pre>
 */
public final class VisionFilter {
    /**
     * An accepted vision measurement.
     *
     * @param pose the robot pose
     * @param timestampSeconds when the image was taken (FPGA time)
     * @param stdDevs x, y (meters) and heading (radians) standard deviations
     */
    public record VisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}

    private double maxAmbiguity = 0.2;
    private double maxDistance = 5.0;
    private double maxZError = 0.3;
    private double fieldLength = 16.541;
    private double fieldWidth = 8.069;
    private double linearStdDev = 0.03;
    private double angularStdDev = 0.06;
    private double singleTagAngularTrust = Double.POSITIVE_INFINITY;
    private double cameraTrust = 1.0;

    /** Maximum pose ambiguity for single-tag estimates (0 to 1). Default 0.2. */
    public VisionFilter maxAmbiguity(double ambiguity) {
        this.maxAmbiguity = ambiguity;
        return this;
    }

    /** Reject estimates whose average tag distance is farther than this, meters. Default 5. */
    public VisionFilter maxDistance(double meters) {
        this.maxDistance = meters;
        return this;
    }

    /** Reject estimates whose height is further than this from the floor, meters. Default 0.3. */
    public VisionFilter maxZError(double meters) {
        this.maxZError = meters;
        return this;
    }

    /** Field size used for the off-field check. Defaults to the 2026 field. */
    public VisionFilter fieldSize(double length, double width) {
        this.fieldLength = length;
        this.fieldWidth = width;
        return this;
    }

    /**
     * Standard deviations for one tag seen from 1 meter. They scale with distance² / tag count.
     * Defaults 0.03 m and 0.06 rad.
     *
     * @param linear x and y, meters
     * @param angular heading, radians
     * @return this, for chaining
     */
    public VisionFilter baseStdDevs(double linear, double angular) {
        this.linearStdDev = linear;
        this.angularStdDev = angular;
        return this;
    }

    /**
     * Multiplies all standard deviations, for a camera you trust less (above 1) or more (below 1).
     * Default 1.
     */
    public VisionFilter cameraTrust(double factor) {
        this.cameraTrust = factor;
        return this;
    }

    /**
     * Whether to use single-tag headings. Single-tag heading is often noisy, so by default it's
     * ignored (infinite standard deviation) and the gyro is used instead.
     */
    public VisionFilter trustSingleTagHeading(boolean trust) {
        this.singleTagAngularTrust = trust ? 1.0 : Double.POSITIVE_INFINITY;
        return this;
    }

    /**
     * Filters one pose estimate.
     *
     * @param estimate the camera's robot pose estimate
     * @param timestampSeconds when the image was taken (FPGA time)
     * @param tagCount number of tags used
     * @param averageTagDistance average distance from the camera to the tags, meters
     * @param ambiguity pose ambiguity (only checked for single-tag estimates; pass 0 if unknown)
     * @return the measurement to use, or empty if it should be ignored
     */
    public Optional<VisionMeasurement> process(
            Pose3d estimate, double timestampSeconds, int tagCount, double averageTagDistance, double ambiguity) {
        if (tagCount <= 0) return Optional.empty();
        if (tagCount == 1 && ambiguity > maxAmbiguity) return Optional.empty();
        if (averageTagDistance > maxDistance) return Optional.empty();
        if (Math.abs(estimate.getZ()) > maxZError) return Optional.empty();
        if (estimate.getX() < 0 || estimate.getX() > fieldLength
                || estimate.getY() < 0 || estimate.getY() > fieldWidth) {
            return Optional.empty();
        }

        double scale = averageTagDistance * averageTagDistance / tagCount * cameraTrust;
        double linear = linearStdDev * scale;
        double angular = angularStdDev * scale * (tagCount == 1 ? singleTagAngularTrust : 1.0);
        return Optional.of(new VisionMeasurement(
                estimate.toPose2d(), timestampSeconds, VecBuilder.fill(linear, linear, angular)));
    }
}
