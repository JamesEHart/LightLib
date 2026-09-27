package io.github.jamesehart.lightutil.math;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Pose and translation helpers on top of WPILib's geometry classes. */
public final class GeomUtil {
    private GeomUtil() {}

    /** Straight-line distance from the robot to a point, in meters. */
    public static double distanceTo(Pose2d pose, Translation2d point) {
        return pose.getTranslation().getDistance(point);
    }

    /** Field-relative direction from the robot to a point. */
    public static Rotation2d angleTo(Pose2d pose, Translation2d point) {
        return point.minus(pose.getTranslation()).getAngle();
    }

    /**
     * Predicts where the robot will be after {@code dtSeconds} if it keeps its current speeds.
     *
     * @param pose current pose
     * @param fieldSpeeds field-relative speeds
     * @param dtSeconds how far ahead to predict
     * @return the predicted pose
     */
    public static Pose2d extrapolate(Pose2d pose, ChassisSpeeds fieldSpeeds, double dtSeconds) {
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation());
        return pose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * dtSeconds,
                robotSpeeds.vyMetersPerSecond * dtSeconds,
                robotSpeeds.omegaRadiansPerSecond * dtSeconds));
    }

    /**
     * Returns true if two poses are within a distance and angle of each other.
     *
     * @param a first pose
     * @param b second pose
     * @param positionTolerance meters
     * @param angleTolerance radians
     * @return whether they're close
     */
    public static boolean isNear(Pose2d a, Pose2d b, double positionTolerance, double angleTolerance) {
        return a.getTranslation().getDistance(b.getTranslation()) <= positionTolerance
                && Math.abs(a.getRotation().minus(b.getRotation()).getRadians()) <= angleTolerance;
    }

    /**
     * Returns the pose in {@code poses} closest (by distance) to {@code pose}, e.g. the nearest
     * scoring location.
     *
     * @param pose the robot pose
     * @param poses candidates; must not be empty
     * @return the closest candidate
     */
    public static Pose2d nearest(Pose2d pose, List<Pose2d> poses) {
        return pose.nearest(poses);
    }
}
