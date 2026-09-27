package io.github.jamesehart.lightutil;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import io.github.jamesehart.lightutil.field.AllianceFlip;
import io.github.jamesehart.lightutil.targeting.AimSolver;
import io.github.jamesehart.lightutil.targeting.ShootOnTheMove;
import io.github.jamesehart.lightutil.targeting.ShotTable;

class TargetingTest {
    private static final double EPS = 1e-6;
    private static final ShotTable TABLE = new ShotTable()
            .add(1, 2000, 0.2, 0.3)
            .add(5, 4000, 0.6, 1.1);

    @AfterEach
    void resetSymmetry() {
        AllianceFlip.setSymmetry(AllianceFlip.Symmetry.ROTATE);
    }

    @Test
    void flipsByRotation() {
        AllianceFlip.setFieldSize(16, 8);
        Pose2d flipped = AllianceFlip.flip(new Pose2d(1, 2, Rotation2d.fromDegrees(30)));
        assertEquals(15, flipped.getX(), EPS);
        assertEquals(6, flipped.getY(), EPS);
        assertEquals(-150, flipped.getRotation().getDegrees(), EPS);
    }

    @Test
    void flipsByMirror() {
        AllianceFlip.setFieldSize(16, 8);
        AllianceFlip.setSymmetry(AllianceFlip.Symmetry.MIRROR);
        Pose2d flipped = AllianceFlip.flip(new Pose2d(1, 2, Rotation2d.fromDegrees(30)));
        assertEquals(15, flipped.getX(), EPS);
        assertEquals(2, flipped.getY(), EPS);
        assertEquals(150, flipped.getRotation().getDegrees(), EPS);
    }

    @Test
    void aimsAtTarget() {
        Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
        Translation2d target = new Translation2d(3, 3);
        assertEquals(Math.hypot(3, 3), AimSolver.distance(robot, target), EPS);
        assertEquals(45, AimSolver.fieldAngle(robot, target).getDegrees(), EPS);
        assertEquals(-45, AimSolver.robotRelativeAngle(robot, target).getDegrees(), EPS);
    }

    @Test
    void turretAccountsForPivotOffset() {
        // Turret 1 m behind center; target 2 m to the left of the pivot.
        Pose2d robot = new Pose2d(1, 0, Rotation2d.kZero);
        double angle = AimSolver.turretAngle(
                robot, new Translation2d(-1, 0), new Translation2d(0, 2), 0, -Math.PI, Math.PI);
        assertEquals(Math.PI / 2, angle, EPS);
    }

    @Test
    void turretFeedforwardCancelsRotationAndStrafe() {
        Translation2d target = new Translation2d(2, 0);
        // Spinning in place: the turret counter-rotates.
        assertEquals(-1, AimSolver.turretVelocityFeedforward(Pose2d.kZero, new ChassisSpeeds(0, 0, 1), target), EPS);
        // Strafing left past a target ahead: its bearing swings right (negative).
        assertEquals(-0.5, AimSolver.turretVelocityFeedforward(Pose2d.kZero, new ChassisSpeeds(0, 1, 0), target), EPS);
    }

    @Test
    void stationaryShotAimsDirectly() {
        Translation2d target = new Translation2d(3, 0);
        ShootOnTheMove.Solution s = ShootOnTheMove.solve(Pose2d.kZero, new ChassisSpeeds(), target, TABLE);
        assertEquals(target, s.virtualTarget());
        assertEquals(3000, s.shot().rpm(), EPS);
    }

    @Test
    void movingShotLeadsTarget() {
        Translation2d target = new Translation2d(3, 0);
        // Driving left at 2 m/s: aim right of the target, by velocity × time of flight.
        ShootOnTheMove.Solution s = ShootOnTheMove.solve(Pose2d.kZero, new ChassisSpeeds(0, 2, 0), target, TABLE);
        assertTrue(s.virtualTarget().getY() < 0);
        assertTrue(s.fieldAngle().getDegrees() < 0);
        double tof = s.shot().timeOfFlight();
        assertEquals(-2 * tof, s.virtualTarget().getY(), 1e-3);
        assertEquals(TABLE.get(s.distance()).timeOfFlight(), tof, 1e-3);
    }
}
