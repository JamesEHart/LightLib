package io.github.jamesehart.lightutil.input;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * More ways to use one button: double tap, tap vs. hold, and two-button chords.
 *
 * <pre>{@code
 * ButtonPatterns.doubleTap(controller.a(), 0.3).onTrue(scoreHigh());
 * ButtonPatterns.tap(controller.b(), 0.4).onTrue(toggleIntake());
 * ButtonPatterns.hold(controller.b(), 0.4).onTrue(ejectEverything());
 * ButtonPatterns.chord(controller.leftBumper(), controller.rightBumper()).onTrue(climb());
 * }</pre>
 *
 * <p>Each returned trigger keeps its own state, so bind it where you create it rather than sharing
 * one across several bindings.
 */
public final class ButtonPatterns {
    private ButtonPatterns() {}

    /**
     * True while the button is held after being pressed twice within {@code windowSeconds}.
     *
     * @param button the button
     * @param windowSeconds max time between the two presses, e.g. 0.3
     * @return the double-tap trigger
     */
    public static Trigger doubleTap(Trigger button, double windowSeconds) {
        return new Trigger(new BooleanSupplier() {
            private boolean lastPressed = false;
            private double lastPressTime = Double.NEGATIVE_INFINITY;
            private boolean active = false;

            @Override
            public boolean getAsBoolean() {
                boolean pressed = button.getAsBoolean();
                double now = Timer.getFPGATimestamp();
                if (pressed && !lastPressed) {
                    active = now - lastPressTime <= windowSeconds;
                    // A triple tap is one double tap plus a fresh first tap, not two double taps.
                    lastPressTime = active ? Double.NEGATIVE_INFINITY : now;
                }
                if (!pressed) active = false;
                lastPressed = pressed;
                return active;
            }
        });
    }

    /**
     * Becomes true once the button has been held for {@code seconds}, and stays true until released.
     *
     * @param button the button
     * @param seconds how long to hold
     * @return the long-press trigger
     */
    public static Trigger hold(Trigger button, double seconds) {
        return button.debounce(seconds, DebounceType.kRising);
    }

    /**
     * True for one loop when the button is released after being held for less than {@code
     * seconds}. Pair with {@link #hold} for different tap and hold actions on one button.
     *
     * @param button the button
     * @param seconds the longest press that still counts as a tap
     * @return the tap trigger
     */
    public static Trigger tap(Trigger button, double seconds) {
        return new Trigger(new BooleanSupplier() {
            private boolean lastPressed = false;
            private double pressTime = 0;

            @Override
            public boolean getAsBoolean() {
                boolean pressed = button.getAsBoolean();
                double now = Timer.getFPGATimestamp();
                boolean fired = false;
                if (pressed && !lastPressed) pressTime = now;
                if (!pressed && lastPressed) fired = now - pressTime < seconds;
                lastPressed = pressed;
                return fired;
            }
        });
    }

    /** True while both buttons are held. */
    public static Trigger chord(Trigger a, Trigger b) {
        return a.and(b);
    }
}
