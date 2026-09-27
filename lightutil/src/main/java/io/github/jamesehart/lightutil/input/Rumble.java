package io.github.jamesehart.lightutil.input;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * Controller rumble as commands, so the driver can feel events like "got a game piece".
 *
 * <pre>{@code
 * hasGamePiece.onTrue(Rumble.rumble(controller, 0.8, 0.3));
 * }</pre>
 *
 * <p>These commands require no subsystems, so they never interrupt anything.
 */
public final class Rumble {
    private Rumble() {}

    /**
     * Rumbles a controller for a fixed time.
     *
     * @param controller the controller
     * @param strength 0 to 1
     * @param seconds how long
     * @return the command
     */
    public static Command rumble(GenericHID controller, double strength, double seconds) {
        return Commands.startEnd(
                        () -> controller.setRumble(RumbleType.kBothRumble, strength),
                        () -> controller.setRumble(RumbleType.kBothRumble, 0))
                .withTimeout(seconds)
                .ignoringDisable(true)
                .withName("Rumble");
    }

    /** Rumbles a command-based controller for a fixed time. See {@link #rumble(GenericHID, double, double)}. */
    public static Command rumble(CommandGenericHID controller, double strength, double seconds) {
        return rumble(controller.getHID(), strength, seconds);
    }

    /**
     * Rumbles in {@code count} short pulses, e.g. 2 pulses for "ready to shoot".
     *
     * @param controller the controller
     * @param strength 0 to 1
     * @param count how many pulses
     * @return the command
     */
    public static Command pulses(CommandGenericHID controller, double strength, int count) {
        return Commands.sequence(rumble(controller, strength, 0.15), Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(count * 0.25 - 0.05)
                .ignoringDisable(true)
                .withName("RumblePulses");
    }
}
