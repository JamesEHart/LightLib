package io.github.jamesehart.lightutil.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Aims while driving. A game piece leaves the shooter with the robot's velocity, so it lands
 * {@code velocity × timeOfFlight} away from where it was aimed. This finds a "virtual target"
 * shifted the opposite way, so aiming at the virtual target hits the real one.
 *
 * <pre>{@code
 * ShootOnTheMove.Solution s = ShootOnTheMove.solve(pose, fieldSpeeds, goal.get2d(), shotTable);
 * drive.faceAngle(s.fieldAngle());
 * shooter.setRpm(s.shot().rpm());
 * }</pre>
 */
public final class ShootOnTheMove {
    private static final int ITERATIONS = 5;

    /**
     * The result of a solve.
     *
     * @param virtualTarget where to aim
     * @param distance distance to the virtual target, meters
     * @param fieldAngle field heading from the robot to the virtual target
     * @param shot shot settings for {@code distance}
     */
    public record Solution(Translation2d virtualTarget, double distance, Rotation2d fieldAngle, ShotTable.Shot shot) {}

    private ShootOnTheMove() {}

    /**
     * Finds the virtual target to aim at while moving.
     *
     * @param robot robot pose (ideally predicted forward by your latency)
     * @param fieldSpeeds the robot's field-relative speeds
     * @param target the real target
     * @param table shot table with times of flight
     * @return the solution
     */
    public static Solution solve(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation2d target, ShotTable table) {
        Translation2d velocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d virtualTarget = target;
        double distance = AimSolver.distance(robot, target);
        ShotTable.Shot shot = table.get(distance);
        // Time of flight depends on distance, which depends on the virtual target: iterate.
        for (int i = 0; i < ITERATIONS; i++) {
            virtualTarget = target.minus(velocity.times(shot.timeOfFlight()));
            distance = AimSolver.distance(robot, virtualTarget);
            shot = table.get(distance);
        }
        return new Solution(virtualTarget, distance, AimSolver.fieldAngle(robot, virtualTarget), shot);
    }
}
