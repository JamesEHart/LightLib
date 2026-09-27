package io.github.jamesehart.lightutil.targeting;

import io.github.jamesehart.lightutil.math.InterpolatingTable;

/**
 * A shooter lookup table: distance to the target → flywheel speed, hood angle and time of flight.
 * Measure a handful of distances on the real robot and the table interpolates between them.
 *
 * <pre>{@code
 * ShotTable table = new ShotTable()
 *     .add(1.5, 2800, Math.toRadians(20), 0.45)
 *     .add(3.0, 3400, Math.toRadians(32), 0.70)
 *     .add(5.0, 4300, Math.toRadians(41), 1.05);
 * ShotTable.Shot shot = table.get(distance);
 * }</pre>
 */
public final class ShotTable {
    /**
     * One shot's settings.
     *
     * @param rpm flywheel speed (or whatever unit you put in the table)
     * @param hoodRad hood angle in radians (or 0 if you have no hood)
     * @param timeOfFlight seconds from leaving the shooter to reaching the target
     */
    public record Shot(double rpm, double hoodRad, double timeOfFlight) {}

    private final InterpolatingTable table = new InterpolatingTable();

    /**
     * Adds a measured shot.
     *
     * @param distance meters to the target
     * @param rpm flywheel speed
     * @param hoodRad hood angle, radians
     * @param timeOfFlight seconds; only needed for shooting on the move, otherwise pass 0
     * @return this table, for chaining
     */
    public ShotTable add(double distance, double rpm, double hoodRad, double timeOfFlight) {
        table.add(distance, rpm, hoodRad, timeOfFlight);
        return this;
    }

    /** The interpolated shot for a distance, clamped to the table's range. */
    public Shot get(double distance) {
        double[] out = table.get(distance);
        return new Shot(out[0], out[1], out[2]);
    }
}
