package io.github.jamesehart.lightutil.auto;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.jamesehart.lightutil.field.AllianceFlip;

/**
 * Checks, before the match, that the robot is sitting where the selected auto expects to start.
 * A robot placed a little off makes the auto drive to its start first (or miss everything), and
 * nobody notices until it's too late.
 *
 * <p>While disabled, it compares the robot's pose (from vision, since odometry doesn't know where
 * it was placed) with the auto's start pose, and shows an alert with how far off it is. Use {@link
 * #ready()} to light up LEDs so the drive team can see it from the field.
 *
 * <pre>{@code
 * AutoStartChecker startCheck = new AutoStartChecker(
 *     drive::getPose,
 *     () -> Optional.ofNullable(autoStartPoses.get(autoChooser.getSelected())));  // blue-side pose
 * startCheck.ready().and(DriverStation::isDisabled)
 *     .whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 5));
 * }</pre>
 */
public final class AutoStartChecker {
    private final Supplier<Pose2d> robotPose;
    private final Supplier<Optional<Pose2d>> startPose;
    private final Alert alert = new Alert("", AlertType.kWarning);
    private double positionTolerance = 0.1;
    private double angleTolerance = Math.toRadians(5);
    private boolean flipForRed = true;
    private double distanceError = 0;
    private double angleError = 0;
    private boolean ready = true;

    /**
     * Creates the checker. It runs automatically every loop through the command scheduler.
     *
     * @param robotPose the robot's estimated pose
     * @param startPose the selected auto's starting pose from the blue side, or empty if it has none
     *     (e.g. "do nothing")
     */
    public AutoStartChecker(Supplier<Pose2d> robotPose, Supplier<Optional<Pose2d>> startPose) {
        this.robotPose = robotPose;
        this.startPose = startPose;
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(this::update);
    }

    /**
     * How close counts as in place. Defaults are 10 cm and 5°.
     *
     * @param meters position tolerance
     * @param radians heading tolerance
     * @return this, for chaining
     */
    public AutoStartChecker withTolerance(double meters, double radians) {
        this.positionTolerance = meters;
        this.angleTolerance = radians;
        return this;
    }

    /**
     * Whether start poses are blue-side and should be flipped on red (the default). Pass false if
     * your start poses are already flipped for the current alliance.
     */
    public AutoStartChecker flipForRed(boolean flip) {
        this.flipForRed = flip;
        return this;
    }

    private void update() {
        Optional<Pose2d> start = startPose.get();
        if (!DriverStation.isDisabled() || start.isEmpty()) {
            ready = true;
            distanceError = 0;
            angleError = 0;
            alert.set(false);
            return;
        }
        Pose2d target = flipForRed ? AllianceFlip.ifRed(start.get()) : start.get();
        Pose2d pose = robotPose.get();
        distanceError = pose.getTranslation().getDistance(target.getTranslation());
        angleError = Math.abs(pose.getRotation().minus(target.getRotation()).getRadians());
        ready = distanceError <= positionTolerance && angleError <= angleTolerance;
        if (!ready) {
            alert.setText(String.format(
                    "Robot is %.2f m / %.0f° from the auto's start pose", distanceError, Math.toDegrees(angleError)));
        }
        alert.set(!ready);
    }

    /** True while the robot is at the selected auto's start (or the auto has no start pose, or we're enabled). */
    public Trigger ready() {
        return new Trigger(() -> ready);
    }

    /** Distance from the start pose at the last check, meters. */
    public double getDistanceError() {
        return distanceError;
    }

    /** Heading difference from the start pose at the last check, radians. */
    public double getAngleError() {
        return angleError;
    }
}
