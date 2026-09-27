package io.github.jamesehart.lightutil.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Converts blue-alliance field positions to the red alliance's side.
 *
 * <p>Write every field position once, from the blue alliance's point of view (WPILib's field origin
 * is the blue corner), and wrap it in {@link #ifRed} when you use it.
 *
 * <p>How the field is flipped depends on the year's field: {@link Symmetry#ROTATE} (the default,
 * right for 2026 REBUILT) turns it 180° about the center, {@link Symmetry#MIRROR} reflects it across
 * the center line.
 */
public final class AllianceFlip {
    /** How the red side relates to the blue side. */
    public enum Symmetry {
        /** Red is blue rotated 180° about the field center (2025, 2026). */
        ROTATE,
        /** Red is blue mirrored across the center line, x → length - x (2023, 2024). */
        MIRROR
    }

    private static Symmetry symmetry = Symmetry.ROTATE;
    private static double fieldLength = Double.NaN;
    private static double fieldWidth = Double.NaN;

    private AllianceFlip() {}

    /** Sets how the field is flipped. Defaults to {@link Symmetry#ROTATE}. */
    public static void setSymmetry(Symmetry newSymmetry) {
        symmetry = newSymmetry;
    }

    /**
     * Overrides the field size. By default it's read from WPILib's default AprilTag field layout.
     *
     * @param length meters, along x
     * @param width meters, along y
     */
    public static void setFieldSize(double length, double width) {
        fieldLength = length;
        fieldWidth = width;
    }

    /** Field length in meters (along x). */
    public static double getFieldLength() {
        loadFieldSize();
        return fieldLength;
    }

    /** Field width in meters (along y). */
    public static double getFieldWidth() {
        loadFieldSize();
        return fieldWidth;
    }

    private static void loadFieldSize() {
        if (!Double.isNaN(fieldLength)) return;
        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            setFieldSize(layout.getFieldLength(), layout.getFieldWidth());
        } catch (RuntimeException | LinkageError e) {
            setFieldSize(16.541, 8.069);
        }
    }

    /** Returns true if the Driver Station says we're on the red alliance. */
    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    /** Flips a blue-side translation to the red side. */
    public static Translation2d flip(Translation2d t) {
        return switch (symmetry) {
            case ROTATE -> new Translation2d(getFieldLength() - t.getX(), getFieldWidth() - t.getY());
            case MIRROR -> new Translation2d(getFieldLength() - t.getX(), t.getY());
        };
    }

    /** Flips a blue-side translation to the red side. Height is unchanged. */
    public static Translation3d flip(Translation3d t) {
        Translation2d flipped = flip(t.toTranslation2d());
        return new Translation3d(flipped.getX(), flipped.getY(), t.getZ());
    }

    /** Flips a blue-side field heading to the red side. */
    public static Rotation2d flip(Rotation2d r) {
        return switch (symmetry) {
            case ROTATE -> r.rotateBy(Rotation2d.kPi);
            case MIRROR -> Rotation2d.kPi.minus(r);
        };
    }

    /** Flips a blue-side pose to the red side. */
    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    /** Returns the translation flipped if we're on red, otherwise unchanged. */
    public static Translation2d ifRed(Translation2d t) {
        return isRed() ? flip(t) : t;
    }

    /** Returns the translation flipped if we're on red, otherwise unchanged. */
    public static Translation3d ifRed(Translation3d t) {
        return isRed() ? flip(t) : t;
    }

    /** Returns the heading flipped if we're on red, otherwise unchanged. */
    public static Rotation2d ifRed(Rotation2d r) {
        return isRed() ? flip(r) : r;
    }

    /** Returns the pose flipped if we're on red, otherwise unchanged. */
    public static Pose2d ifRed(Pose2d pose) {
        return isRed() ? flip(pose) : pose;
    }
}
