package io.github.jamesehart.lightutil.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import io.github.jamesehart.lightutil.math.AngleUtil;

/**
 * Angles and distances from the robot to a target, for aiming the drivetrain or a turret.
 *
 * <p>Pass targets as field positions, e.g. {@code target.get2d()} for a {@link Target}. For
 * latency compensation, pass a predicted pose ({@code GeomUtil.extrapolate}) instead of the current
 * one.
 */
public final class AimSolver {
    private AimSolver() {}

    /** Distance from the robot's center to the target on the floor, in meters. */
    public static double distance(Pose2d robot, Translation2d target) {
        return robot.getTranslation().getDistance(target);
    }

    /** The field heading the robot should face to point its front at the target. */
    public static Rotation2d fieldAngle(Pose2d robot, Translation2d target) {
        return target.minus(robot.getTranslation()).getAngle();
    }

    /** The target's direction relative to the robot's front (positive is to the left). */
    public static Rotation2d robotRelativeAngle(Pose2d robot, Translation2d target) {
        return fieldAngle(robot, target).minus(robot.getRotation());
    }

    /**
     * The turret angle (relative to the robot's front, counter-clockwise positive) that points the
     * turret at the target, within the turret's legal range and taking the shortest path from its
     * current angle.
     *
     * @param robot robot pose
     * @param robotToTurret where the turret's pivot is on the robot (x forward, y left)
     * @param target target position
     * @param currentRad the turret's current angle, radians
     * @param minRad lowest legal turret angle, radians
     * @param maxRad highest legal turret angle, radians
     * @return the turret angle to command, radians
     */
    public static double turretAngle(
            Pose2d robot,
            Translation2d robotToTurret,
            Translation2d target,
            double currentRad,
            double minRad,
            double maxRad) {
        Translation2d pivot = robot.getTranslation().plus(robotToTurret.rotateBy(robot.getRotation()));
        double desired = target.minus(pivot).getAngle().minus(robot.getRotation()).getRadians();
        return AngleUtil.placeInRange(desired, currentRad, minRad, maxRad);
    }

    /**
     * The turret velocity (rad/s, relative to the robot) that keeps it pointed at the target while
     * the robot drives and turns. Add it to the turret's feedforward so it doesn't lag behind.
     *
     * @param robot robot pose
     * @param fieldSpeeds the robot's field-relative speeds
     * @param target target position
     * @return turret velocity feedforward, rad/s
     */
    public static double turretVelocityFeedforward(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation2d target) {
        Translation2d r = target.minus(robot.getTranslation());
        double distSq = r.getX() * r.getX() + r.getY() * r.getY();
        if (distSq < 1e-6) return -fieldSpeeds.omegaRadiansPerSecond;
        // The target's bearing changes as the robot moves sideways to it; the chassis turning
        // has to be undone on top of that.
        double bearingRate = -(r.getX() * fieldSpeeds.vyMetersPerSecond - r.getY() * fieldSpeeds.vxMetersPerSecond) / distSq;
        return bearingRate - fieldSpeeds.omegaRadiansPerSecond;
    }
}
