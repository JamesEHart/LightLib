package io.github.jamesehart.lightutil;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import io.github.jamesehart.lightutil.drive.SkidDetector;
import io.github.jamesehart.lightutil.input.InputCurves;
import io.github.jamesehart.lightutil.vision.ObjectDetection;
import io.github.jamesehart.lightutil.vision.VisionFilter;

class VisionAndDriveTest {
    private static final double EPS = 1e-6;
    private static final Pose3d ON_FIELD = new Pose3d(5, 4, 0, Rotation3d.kZero);

    @Test
    void visionRejectsBadEstimates() {
        VisionFilter filter = new VisionFilter();
        assertTrue(filter.process(ON_FIELD, 0, 2, 2, 0).isPresent());
        assertFalse(filter.process(ON_FIELD, 0, 1, 2, 0.5).isPresent(), "ambiguous");
        assertFalse(filter.process(ON_FIELD, 0, 2, 8, 0).isPresent(), "too far");
        assertFalse(filter.process(new Pose3d(5, 4, 1, Rotation3d.kZero), 0, 2, 2, 0).isPresent(), "floating");
        assertFalse(filter.process(new Pose3d(-1, 4, 0, Rotation3d.kZero), 0, 2, 2, 0).isPresent(), "off field");
        assertFalse(filter.process(ON_FIELD, 0, 0, 2, 0).isPresent(), "no tags");
    }

    @Test
    void visionTrustsCloseMultiTagMore() {
        VisionFilter filter = new VisionFilter().baseStdDevs(0.1, 0.1);
        double close = filter.process(ON_FIELD, 0, 2, 1, 0).orElseThrow().stdDevs().get(0, 0);
        double far = filter.process(ON_FIELD, 0, 2, 3, 0).orElseThrow().stdDevs().get(0, 0);
        double single = filter.process(ON_FIELD, 0, 1, 1, 0).orElseThrow().stdDevs().get(0, 0);
        assertEquals(0.05, close, EPS);
        assertEquals(9 * close, far, EPS);
        assertEquals(2 * close, single, EPS);
        assertEquals(Double.POSITIVE_INFINITY,
                filter.process(ON_FIELD, 0, 1, 1, 0).orElseThrow().stdDevs().get(2, 0));
    }

    @Test
    void objectDetectionHitsFloor() {
        // Camera 1 m up, tilted 45° down, on a robot at (2, 3) facing +y.
        Transform3d robotToCamera = new Transform3d(0, 0, 1, new Rotation3d(0, Math.PI / 4, 0));
        Pose2d robot = new Pose2d(2, 3, Rotation2d.kCCW_Pi_2);
        Translation2d piece = ObjectDetection.toField(robot, robotToCamera, 0, 0, 0).orElseThrow();
        assertEquals(2, piece.getX(), EPS);
        assertEquals(4, piece.getY(), EPS);
        // Looking above the horizon never reaches the floor.
        assertFalse(ObjectDetection.toField(robot, robotToCamera, 0, Math.toRadians(50), 0).isPresent());
    }

    @Test
    void skidDetectorFlagsOneFastWheel() {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
                new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3));
        SkidDetector skid = new SkidDetector(kinematics, 0.3);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(1, 0.5, 2));
        assertEquals(0, skid.update(states), 1e-9);
        assertFalse(skid.isSkidding());

        states[0] = new SwerveModuleState(states[0].speedMetersPerSecond + 1.5, states[0].angle);
        skid.update(states);
        assertTrue(skid.isSkidding());
        double[] errors = skid.getModuleErrors();
        for (int i = 1; i < 4; i++) assertTrue(errors[0] > errors[i]);
    }

    @Test
    void stickShapingKeepsDirection() {
        Translation2d full = InputCurves.stick(1, 1, 0.1, 2);
        assertEquals(1, full.getNorm(), EPS);
        assertEquals(45, full.getAngle().getDegrees(), EPS);
        assertEquals(Translation2d.kZero, InputCurves.stick(0.05, 0.05, 0.1, 2));
        assertEquals(-0.25, InputCurves.power(-0.5, 2), EPS);
    }
}
