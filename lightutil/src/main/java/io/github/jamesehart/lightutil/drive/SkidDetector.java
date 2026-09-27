package io.github.jamesehart.lightutil.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Detects swerve wheels slipping. If every wheel grips, the measured module velocities all agree
 * with one rigid-body motion of the robot. A wheel that disagrees is slipping (or being pushed).
 * Use it to distrust odometry and lean on vision while skidding, or to warn the driver.
 *
 * <pre>{@code
 * SkidDetector skid = new SkidDetector(kinematics, 0.5);
 * // in periodic():
 * skid.update(getModuleStates());
 * if (skid.isSkidding()) { ... }
 * }</pre>
 */
public final class SkidDetector {
    private final SwerveDriveKinematics kinematics;
    private final double threshold;
    private final double[] errors;
    private double maxError = 0;

    /**
     * Creates a skid detector.
     *
     * @param kinematics the drivetrain's kinematics
     * @param thresholdMetersPerSecond how far a wheel's velocity can be from the robot's motion
     *     before it counts as skidding, e.g. 0.5
     */
    public SkidDetector(SwerveDriveKinematics kinematics, double thresholdMetersPerSecond) {
        this.kinematics = kinematics;
        this.threshold = thresholdMetersPerSecond;
        this.errors = new double[kinematics.getModules().length];
    }

    /**
     * Updates with the latest measured module states. Call every loop.
     *
     * @param measured measured module states, in the same order as the kinematics
     * @return the largest module error, m/s
     */
    public double update(SwerveModuleState... measured) {
        // The best-fit rigid-body motion, and what each wheel would read if it followed it.
        ChassisSpeeds fit = kinematics.toChassisSpeeds(measured);
        SwerveModuleState[] expected = kinematics.toSwerveModuleStates(fit);
        maxError = 0;
        for (int i = 0; i < measured.length; i++) {
            Translation2d error = velocity(measured[i]).minus(velocity(expected[i]));
            errors[i] = error.getNorm();
            maxError = Math.max(maxError, errors[i]);
        }
        return maxError;
    }

    private static Translation2d velocity(SwerveModuleState state) {
        return new Translation2d(state.speedMetersPerSecond, state.angle);
    }

    /** True if any wheel's error was over the threshold at the last update. */
    public boolean isSkidding() {
        return maxError > threshold;
    }

    /** The largest module error at the last update, m/s. */
    public double getMaxError() {
        return maxError;
    }

    /** Each module's error at the last update, m/s. */
    public double[] getModuleErrors() {
        return errors.clone();
    }
}
