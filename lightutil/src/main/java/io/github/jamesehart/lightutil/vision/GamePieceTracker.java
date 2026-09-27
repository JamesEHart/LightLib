package io.github.jamesehart.lightutil.vision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Remembers where game pieces are on the field from object detection, so the robot can drive to
 * one even when the camera briefly loses it (motion blur, another robot in the way, turning away).
 *
 * <p>Each detection either updates a piece already being tracked nearby or adds a new one. Pieces
 * not seen for a while are forgotten, and pieces seen only once can be ignored as noise.
 *
 * <pre>{@code
 * GamePieceTracker pieces = new GamePieceTracker();
 * // for each detection in the camera result:
 * ObjectDetection.toField(pose, robotToCamera, yaw, pitch, 0.07)
 *     .ifPresent(p -> pieces.add(p, result.getTimestampSeconds()));
 * // in periodic():
 * pieces.removeOlderThan(Timer.getFPGATimestamp());
 * Optional<Translation2d> target = pieces.closest(drive.getPose());
 * }</pre>
 */
public final class GamePieceTracker {
    /**
     * A tracked piece.
     *
     * @param position field position, meters
     * @param lastSeen timestamp of the latest detection, seconds
     * @param sightings how many detections have matched it
     */
    public record Piece(Translation2d position, double lastSeen, int sightings) {}

    private final List<Piece> pieces = new ArrayList<>();
    private double mergeDistance = 0.3;
    private double forgetSeconds = 1.0;
    private int minSightings = 2;
    private double smoothing = 0.5;

    /** Detections closer than this to a tracked piece count as the same piece, meters. Default 0.3. */
    public GamePieceTracker mergeDistance(double meters) {
        this.mergeDistance = meters;
        return this;
    }

    /** Forget a piece not seen for this long, seconds. Default 1. */
    public GamePieceTracker forgetAfter(double seconds) {
        this.forgetSeconds = seconds;
        return this;
    }

    /** A piece must be seen this many times before it's reported, to filter out false detections. Default 2. */
    public GamePieceTracker minSightings(int count) {
        this.minSightings = count;
        return this;
    }

    /**
     * How much a new detection moves a tracked piece: 1 jumps straight to the new detection, lower
     * values average out noise. Default 0.5.
     */
    public GamePieceTracker smoothing(double weight) {
        this.smoothing = weight;
        return this;
    }

    /**
     * Adds a detection.
     *
     * @param position the piece's field position (e.g. from {@link ObjectDetection#toField})
     * @param timestamp when the image was taken, seconds
     */
    public void add(Translation2d position, double timestamp) {
        int nearest = -1;
        double nearestDistance = mergeDistance;
        for (int i = 0; i < pieces.size(); i++) {
            double d = pieces.get(i).position().getDistance(position);
            if (d <= nearestDistance) {
                nearest = i;
                nearestDistance = d;
            }
        }
        if (nearest == -1) {
            pieces.add(new Piece(position, timestamp, 1));
        } else {
            Piece old = pieces.get(nearest);
            Translation2d merged = old.position().interpolate(position, smoothing);
            pieces.set(nearest, new Piece(merged, Math.max(old.lastSeen(), timestamp), old.sightings() + 1));
        }
    }

    /** Adds several detections from one image. */
    public void addAll(Collection<Translation2d> positions, double timestamp) {
        for (Translation2d p : positions) add(p, timestamp);
    }

    /** Forgets pieces not seen within the forget time before {@code now}. Call every loop. */
    public void removeOlderThan(double now) {
        pieces.removeIf(p -> now - p.lastSeen() > forgetSeconds);
    }

    /** Forgets pieces within {@code radius} of a point, e.g. the one the robot just picked up. */
    public void removeNear(Translation2d point, double radius) {
        pieces.removeIf(p -> p.position().getDistance(point) <= radius);
    }

    /** Forgets everything. */
    public void clear() {
        pieces.clear();
    }

    /** Confirmed pieces (seen at least the minimum number of times). */
    public List<Piece> getPieces() {
        return pieces.stream().filter(p -> p.sightings() >= minSightings).toList();
    }

    /** Positions of confirmed pieces, e.g. for logging as a {@code Translation2d[]}. */
    public Translation2d[] getPositions() {
        return getPieces().stream().map(Piece::position).toArray(Translation2d[]::new);
    }

    /** The closest confirmed piece to the robot. */
    public Optional<Translation2d> closest(Pose2d robot) {
        return closest(robot, p -> true);
    }

    /**
     * The closest confirmed piece that passes a filter, e.g. only pieces on our side: {@code
     * p -> ourHalf.contains(p)}.
     *
     * @param robot robot pose
     * @param filter which positions to consider
     * @return the closest matching piece
     */
    public Optional<Translation2d> closest(Pose2d robot, Predicate<Translation2d> filter) {
        Translation2d best = null;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (Piece piece : getPieces()) {
            if (!filter.test(piece.position())) continue;
            double d = piece.position().getDistance(robot.getTranslation());
            if (d < bestDistance) {
                best = piece.position();
                bestDistance = d;
            }
        }
        return Optional.ofNullable(best);
    }
}
