package io.github.jamesehart.lightutil.math;

import java.util.Map;
import java.util.TreeMap;

/**
 * A lookup table from one input to several outputs, with linear interpolation between rows. Inputs
 * outside the table are clamped to the first or last row.
 *
 * <pre>{@code
 * InterpolatingTable table = new InterpolatingTable()
 *     .add(2.0, 3000, 0.40)   // distance -> rpm, hood angle
 *     .add(4.0, 3800, 0.60);
 * double[] out = table.get(3.0);  // {3400, 0.50}
 * }</pre>
 */
public final class InterpolatingTable {
    private final TreeMap<Double, double[]> rows = new TreeMap<>();
    private int width = -1;

    /**
     * Adds a row. Every row must have the same number of outputs.
     *
     * @param key the input, e.g. distance
     * @param outputs the outputs for that input
     * @return this table, for chaining
     */
    public InterpolatingTable add(double key, double... outputs) {
        if (width == -1) {
            width = outputs.length;
        } else if (outputs.length != width) {
            throw new IllegalArgumentException(
                    "Expected " + width + " outputs per row, got " + outputs.length);
        }
        rows.put(key, outputs.clone());
        return this;
    }

    /** Number of rows. */
    public int size() {
        return rows.size();
    }

    /**
     * Returns the interpolated outputs for {@code key}.
     *
     * @param key the input
     * @return a new array of outputs
     * @throws IllegalStateException if the table is empty
     */
    public double[] get(double key) {
        if (rows.isEmpty()) throw new IllegalStateException("InterpolatingTable is empty");
        Map.Entry<Double, double[]> below = rows.floorEntry(key);
        Map.Entry<Double, double[]> above = rows.ceilingEntry(key);
        if (below == null) return above.getValue().clone();
        if (above == null || below.getKey().equals(above.getKey())) return below.getValue().clone();

        double t = (key - below.getKey()) / (above.getKey() - below.getKey());
        double[] a = below.getValue();
        double[] b = above.getValue();
        double[] out = new double[width];
        for (int i = 0; i < width; i++) {
            out[i] = a[i] + (b[i] - a[i]) * t;
        }
        return out;
    }

    /**
     * Returns one interpolated output for {@code key}.
     *
     * @param key the input
     * @param index which output column
     * @return the interpolated value
     */
    public double get(double key, int index) {
        return get(key)[index];
    }
}
