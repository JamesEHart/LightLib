package io.github.jamesehart.lightutil.math;

/** Angle helpers. Everything is in radians. */
public final class AngleUtil {
    private static final double TAU = 2 * Math.PI;

    private AngleUtil() {}

    /** Wraps an angle into [-π, π). */
    public static double wrapPi(double angle) {
        return wrap2Pi(angle + Math.PI) - Math.PI;
    }

    /** Wraps an angle into [0, 2π). */
    public static double wrap2Pi(double angle) {
        double wrapped = angle % TAU;
        return wrapped < 0 ? wrapped + TAU : wrapped;
    }

    /** The shortest signed rotation from {@code from} to {@code to}, in [-π, π). */
    public static double shortestDelta(double from, double to) {
        return wrapPi(to - from);
    }

    /**
     * Picks the angle equivalent to {@code target} (target + k·2π) that lies within [min, max] and is
     * closest to {@code current}. Use it for mechanisms with a limited range that may be more than
     * one turn wide, like a turret with a cable chain: it takes the shortest legal path instead of
     * wrapping through the cable limit.
     *
     * <p>If no equivalent angle fits in the range, returns the range limit closest to the target.
     *
     * @param target desired angle
     * @param current mechanism's current angle
     * @param min lowest legal angle
     * @param max highest legal angle
     * @return the angle to command
     */
    public static double placeInRange(double target, double current, double min, double max) {
        // The equivalent angle nearest the current angle, then step by whole turns into range.
        double best = Double.NaN;
        double start = current + shortestDelta(current, target);
        for (int k = -3; k <= 3; k++) {
            double candidate = start + k * TAU;
            if (candidate < min || candidate > max) continue;
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) return best;
        double toMin = Math.abs(shortestDelta(target, min));
        double toMax = Math.abs(shortestDelta(target, max));
        return toMin < toMax ? min : max;
    }
}
