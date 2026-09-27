package io.github.jamesehart.lightutil.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Joystick shaping. Curves make small stick movements gentler for precise control, while full
 * stick still gives full speed. All inputs and outputs are in [-1, 1].
 */
public final class InputCurves {
    private InputCurves() {}

    /** Zero inside the deadband, then rescaled so the output still reaches ±1. */
    public static double deadband(double x, double deadband) {
        return MathUtil.applyDeadband(x, deadband);
    }

    /** {@code sign(x)·|x|^exponent}. 2 (squared) is a common choice for driving. */
    public static double power(double x, double exponent) {
        return Math.copySign(Math.pow(Math.abs(x), exponent), x);
    }

    /**
     * A blend of linear and cubic: {@code k·x³ + (1-k)·x}.
     *
     * @param x input
     * @param k 0 is linear, 1 is fully cubic
     * @return shaped output
     */
    public static double expo(double x, double k) {
        return k * x * x * x + (1 - k) * x;
    }

    /**
     * Shapes a 2D stick by its distance from center, keeping its direction. Unlike shaping x and y
     * separately, this doesn't pull diagonal movement toward the axes, and diagonals top out at 1
     * instead of √2.
     *
     * @param x stick x
     * @param y stick y
     * @param deadband radial deadband
     * @param exponent power curve applied to the magnitude
     * @return shaped (x, y), with magnitude at most 1
     */
    public static Translation2d stick(double x, double y, double deadband, double exponent) {
        double magnitude = Math.hypot(x, y);
        if (magnitude < 1e-9) return Translation2d.kZero;
        double shaped = power(deadband(Math.min(magnitude, 1), deadband), exponent);
        return new Translation2d(x / magnitude * shaped, y / magnitude * shaped);
    }
}
