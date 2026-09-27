package io.github.jamesehart.lightutil.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Turns a camera detection of a game piece on the floor (its angles in the image) into a field
 * position, using where the camera is mounted.
 *
 * <p>Angles use WPILib's convention: yaw positive to the <b>left</b>, pitch positive <b>up</b>.
 * Limelight's {@code tx} and PhotonVision's {@code getYaw()} are positive to the right, so negate
 * them:
 *
 * <pre>{@code
 * ObjectDetection.toField(pose, robotToCamera,
 *     Math.toRadians(-limelight.tx), Math.toRadians(limelight.ty), 0.07)
 *   .ifPresent(piece -> ...);
 * }</pre>
 */
public final class ObjectDetection {
    private ObjectDetection() {}

    /**
     * Finds where a detected object is on the field.
     *
     * @param robotPose robot pose when the image was taken
     * @param robotToCamera camera position and rotation on the robot (pitch positive = tilted down)
     * @param yawRad horizontal angle to the object in the image, positive left
     * @param pitchRad vertical angle to the object in the image, positive up
     * @param objectHeight height of the point being detected (e.g. the center of the piece), meters
     * @return the object's field position, or empty if the ray never reaches that height (e.g. the
     *     object is above the horizon)
     */
    public static Optional<Translation2d> toField(
            Pose2d robotPose, Transform3d robotToCamera, double yawRad, double pitchRad, double objectHeight) {
        // Ray through the pixel for a pinhole camera, then into the robot's frame.
        Translation3d direction = new Translation3d(1, Math.tan(yawRad), Math.tan(pitchRad))
                .rotateBy(robotToCamera.getRotation());
        Translation3d camera = robotToCamera.getTranslation();
        if (Math.abs(direction.getZ()) < 1e-9) return Optional.empty();
        double t = (objectHeight - camera.getZ()) / direction.getZ();
        if (t <= 0) return Optional.empty();

        Translation2d robotRelative = new Translation2d(
                camera.getX() + direction.getX() * t,
                camera.getY() + direction.getY() * t);
        return Optional.of(robotPose.getTranslation().plus(robotRelative.rotateBy(robotPose.getRotation())));
    }
}
