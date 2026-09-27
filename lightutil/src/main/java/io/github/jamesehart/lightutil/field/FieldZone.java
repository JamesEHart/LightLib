package io.github.jamesehart.lightutil.field;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.jamesehart.lightutil.math.Polygon2d;

/**
 * A named area of the field, written from the blue alliance's side and automatically flipped on
 * red. Use {@link #trigger} to run commands when the robot enters or leaves it.
 *
 * <pre>{@code
 * FieldZone scoringZone = new FieldZone("Scoring", Polygon2d.rectangle(
 *     new Translation2d(0, 0), new Translation2d(4, 8.07)));
 * scoringZone.trigger(drive::getPose).whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 1));
 * }</pre>
 */
public final class FieldZone {
    private final String name;
    private final Polygon2d bluePolygon;
    private final boolean flipForRed;
    private Polygon2d redPolygon;

    /**
     * Creates a zone that flips to the red side when we're on red.
     *
     * @param name the zone's name
     * @param bluePolygon the zone on the blue side
     */
    public FieldZone(String name, Polygon2d bluePolygon) {
        this(name, bluePolygon, true);
    }

    /**
     * Creates a zone.
     *
     * @param name the zone's name
     * @param bluePolygon the zone, from the blue side
     * @param flipForRed false for zones that are the same for both alliances (e.g. the center line)
     */
    public FieldZone(String name, Polygon2d bluePolygon, boolean flipForRed) {
        this.name = name;
        this.bluePolygon = bluePolygon;
        this.flipForRed = flipForRed;
    }

    /** The zone's name. */
    public String getName() {
        return name;
    }

    /** The zone for the current alliance. */
    public Polygon2d getPolygon() {
        if (!flipForRed || !AllianceFlip.isRed()) return bluePolygon;
        if (redPolygon == null) redPolygon = bluePolygon.map(AllianceFlip::flip);
        return redPolygon;
    }

    /** Returns true if the point is in the zone (for the current alliance). */
    public boolean contains(Translation2d point) {
        return getPolygon().contains(point);
    }

    /** A trigger that's true while the robot is in the zone. */
    public Trigger trigger(Supplier<Pose2d> robotPose) {
        return new Trigger(() -> contains(robotPose.get().getTranslation()));
    }
}
