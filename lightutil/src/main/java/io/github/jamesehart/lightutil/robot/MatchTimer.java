package io.github.jamesehart.lightutil.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Match time helpers, for warning the driver before the end of the match.
 *
 * <pre>{@code
 * MatchTimer.teleopTimeLeftBelow(20).onTrue(Rumble.pulses(controller, 1, 3));
 * }</pre>
 *
 * <p>Match time comes from the Driver Station. It counts down in real matches and in the Driver
 * Station's practice mode, but not when you just enable teleop, where it's -1.
 */
public final class MatchTimer {
    private MatchTimer() {}

    /** Seconds left in the current period (auto or teleop), or -1 if unknown. */
    public static double timeLeft() {
        return DriverStation.getMatchTime();
    }

    /** True in teleop when {@code seconds} or less are left. */
    public static boolean isTeleopTimeLeftBelow(double seconds) {
        double left = timeLeft();
        return DriverStation.isTeleopEnabled() && left >= 0 && left <= seconds;
    }

    /** True in teleop during the last 30 seconds. */
    public static boolean isEndgame() {
        return isTeleopTimeLeftBelow(30);
    }

    /** A trigger that's true in teleop once {@code seconds} or less are left. */
    public static Trigger teleopTimeLeftBelow(double seconds) {
        return new Trigger(() -> isTeleopTimeLeftBelow(seconds));
    }

    /** A trigger that's true during the last 30 seconds of teleop. */
    public static Trigger endgame() {
        return new Trigger(MatchTimer::isEndgame);
    }
}
