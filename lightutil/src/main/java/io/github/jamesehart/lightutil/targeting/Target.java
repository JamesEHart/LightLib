package io.github.jamesehart.lightutil.targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import io.github.jamesehart.lightutil.field.AllianceFlip;

/**
 * Something on the field to aim at, like a goal. Give its blue-alliance position; {@link #get()}
 * returns the position for whichever alliance we're on.
 *
 * <p>For a target both alliances share, use {@link #neutral}.
 */
public final class Target {
    private final String name;
    private final Translation3d bluePosition;
    private final boolean flipForRed;

    private Target(String name, Translation3d bluePosition, boolean flipForRed) {
        this.name = name;
        this.bluePosition = bluePosition;
        this.flipForRed = flipForRed;
    }

    /**
     * Creates an alliance-specific target, e.g. your own goal.
     *
     * @param name the target's name
     * @param bluePosition where it is for the blue alliance (meters; z is height)
     * @return the target
     */
    public static Target of(String name, Translation3d bluePosition) {
        return new Target(name, bluePosition, true);
    }

    /** Creates an alliance-specific target on the floor (height 0). */
    public static Target of(String name, Translation2d bluePosition) {
        return of(name, new Translation3d(bluePosition.getX(), bluePosition.getY(), 0));
    }

    /** Creates a target that's in the same place for both alliances. */
    public static Target neutral(String name, Translation3d position) {
        return new Target(name, position, false);
    }

    /** The target's name. */
    public String getName() {
        return name;
    }

    /** The target's position for the current alliance. */
    public Translation3d get() {
        return flipForRed ? AllianceFlip.ifRed(bluePosition) : bluePosition;
    }

    /** The target's position on the floor for the current alliance. */
    public Translation2d get2d() {
        return get().toTranslation2d();
    }
}
