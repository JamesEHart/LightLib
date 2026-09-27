package io.github.jamesehart.lightutil.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Triggers that remember state. Like {@link io.github.jamesehart.lightutil.input.ButtonPatterns},
 * each returned trigger keeps its own state, so bind it where you create it.
 */
public final class TriggerUtil {
    private TriggerUtil() {}

    /**
     * Becomes true when {@code set} is true, and stays true until {@code reset} is true.
     *
     * @param set turns it on
     * @param reset turns it off (wins if both are true)
     * @return the latched trigger
     */
    public static Trigger latch(BooleanSupplier set, BooleanSupplier reset) {
        return new Trigger(new BooleanSupplier() {
            private boolean latched = false;

            @Override
            public boolean getAsBoolean() {
                if (set.getAsBoolean()) latched = true;
                if (reset.getAsBoolean()) latched = false;
                return latched;
            }
        });
    }

    /**
     * Flips between true and false each time {@code button} is pressed, e.g. a slow-mode toggle.
     *
     * @param button the button
     * @param initial the starting value
     * @return the toggle trigger
     */
    public static Trigger toggle(BooleanSupplier button, boolean initial) {
        return new Trigger(new BooleanSupplier() {
            private boolean last = false;
            private boolean state = initial;

            @Override
            public boolean getAsBoolean() {
                boolean now = button.getAsBoolean();
                if (now && !last) state = !state;
                last = now;
                return state;
            }
        });
    }

    /** True for one loop when {@code condition} goes from false to true. */
    public static Trigger risingEdge(BooleanSupplier condition) {
        return edge(condition, true);
    }

    /** True for one loop when {@code condition} goes from true to false. */
    public static Trigger fallingEdge(BooleanSupplier condition) {
        return edge(condition, false);
    }

    private static Trigger edge(BooleanSupplier condition, boolean rising) {
        return new Trigger(new BooleanSupplier() {
            private boolean last = false;

            @Override
            public boolean getAsBoolean() {
                boolean now = condition.getAsBoolean();
                boolean fired = rising ? now && !last : !now && last;
                last = now;
                return fired;
            }
        });
    }
}
