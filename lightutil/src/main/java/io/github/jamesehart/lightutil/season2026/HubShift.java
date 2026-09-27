package io.github.jamesehart.lightutil.season2026;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.jamesehart.lightutil.field.AllianceFlip;

/**
 * <b>2026 REBUILT only.</b> Whether your alliance's hub is active, and how long until that changes.
 *
 * <p>Both hubs are active in auto, the 10 s transition shift and the last 30 s. In between are four
 * 25 s alliance shifts where only one hub is active. The FMS sends which alliance's hub goes
 * inactive first (the one that scored more fuel in auto) as game data ('R' or 'B') about 3 seconds
 * after auto. Until then the hub is assumed active.
 *
 * <pre>{@code
 * HubShift.hubActive().onTrue(leds.show(LEDPattern.solid(Color.kGreen), 3));
 * // Fuel takes time to fly: start shooting just before the hub turns on.
 * boolean shoot = HubShift.isHubActiveIn(shot.timeOfFlight());
 * }</pre>
 *
 * <p>Match time from the Driver Station is approximate, so don't block the driver from shooting on
 * this alone. Timings follow the WPILib 2026 game data docs.
 */
public final class HubShift {
    private static final double TRANSITION_END = 130;
    private static final double[] SHIFT_ENDS = {105, 80, 55, 30};

    private HubShift() {}

    /** True if our hub is active right now. */
    public static boolean isHubActive() {
        return isHubActiveIn(0);
    }

    /**
     * True if our hub will be active {@code seconds} from now, e.g. when a shot fired now arrives.
     *
     * @param seconds how far ahead
     * @return whether the hub will be active then
     */
    public static boolean isHubActiveIn(double seconds) {
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;
        double time = DriverStation.getMatchTime();
        if (time < 0) return true;
        return isActive(AllianceFlip.isRed(), DriverStation.getGameSpecificMessage(), time - seconds);
    }

    /**
     * Seconds until our hub switches between active and inactive, or infinity if it won't again
     * this match (or it's unknown).
     */
    public static double timeUntilChange() {
        if (!DriverStation.isTeleopEnabled()) return Double.POSITIVE_INFINITY;
        return timeUntilChange(AllianceFlip.isRed(), DriverStation.getGameSpecificMessage(), DriverStation.getMatchTime());
    }

    /** A trigger that's true while our hub is active. */
    public static Trigger hubActive() {
        return new Trigger(HubShift::isHubActive);
    }

    /**
     * Whether a hub is active at a point in teleop. Doesn't read the Driver Station, so it's usable
     * in tests.
     *
     * @param isRed whether we're on the red alliance
     * @param gameData the game-specific message ("R", "B", or empty before it arrives)
     * @param teleopTimeLeft seconds left in teleop
     * @return whether our hub is active
     */
    public static boolean isActive(boolean isRed, String gameData, double teleopTimeLeft) {
        if (teleopTimeLeft < 0) return false;
        if (teleopTimeLeft > TRANSITION_END || teleopTimeLeft <= SHIFT_ENDS[3]) return true;
        if (gameData == null || gameData.isEmpty()) return true;
        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }
        // Shifts 1 and 3 belong to the alliance whose hub didn't go inactive first.
        boolean activeInOddShifts = isRed != redInactiveFirst;
        int shift = 0;
        while (teleopTimeLeft <= SHIFT_ENDS[shift]) shift++;
        return (shift % 2 == 0) == activeInOddShifts;
    }

    /**
     * Seconds until the hub's state changes. Doesn't read the Driver Station.
     *
     * @param isRed whether we're on the red alliance
     * @param gameData the game-specific message
     * @param teleopTimeLeft seconds left in teleop
     * @return seconds until the next change, or infinity if none
     */
    public static double timeUntilChange(boolean isRed, String gameData, double teleopTimeLeft) {
        if (teleopTimeLeft < 0) return Double.POSITIVE_INFINITY;
        boolean now = isActive(isRed, gameData, teleopTimeLeft);
        double[] boundaries = {TRANSITION_END, SHIFT_ENDS[0], SHIFT_ENDS[1], SHIFT_ENDS[2], SHIFT_ENDS[3]};
        for (double boundary : boundaries) {
            if (boundary >= teleopTimeLeft) continue;
            if (isActive(isRed, gameData, boundary - 1e-3) != now) return teleopTimeLeft - boundary;
        }
        return Double.POSITIVE_INFINITY;
    }
}
