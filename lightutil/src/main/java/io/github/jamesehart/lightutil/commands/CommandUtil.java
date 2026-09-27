package io.github.jamesehart.lightutil.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Command helpers. */
public final class CommandUtil {
    private CommandUtil() {}

    /**
     * Waits until a condition is true, but gives up after a timeout and runs {@code onTimeout}.
     * Stops an auto from hanging forever on a sensor that never triggers.
     *
     * <pre>{@code
     * CommandUtil.waitUntilOrTimeout(intake::hasPiece, 2.0,
     *     () -> DriverStation.reportWarning("Intake timed out", false));
     * }</pre>
     *
     * @param condition what to wait for
     * @param seconds the timeout
     * @param onTimeout runs only if the timeout is reached first
     * @return the command
     */
    public static Command waitUntilOrTimeout(BooleanSupplier condition, double seconds, Runnable onTimeout) {
        return Commands.race(
                        Commands.waitUntil(condition),
                        Commands.waitSeconds(seconds).andThen(Commands.runOnce(onTimeout)))
                .withName("WaitUntilOrTimeout");
    }

    /**
     * Wraps a command so it prints when it starts, finishes or is interrupted, with how long it
     * ran. Messages go to the console and, if it's running, the {@code DataLogManager} log file.
     *
     * @param name name to print
     * @param command the command
     * @return the wrapped command
     */
    public static Command logged(String name, Command command) {
        double[] start = new double[1];
        return command
                .beforeStarting(() -> {
                    start[0] = Timer.getFPGATimestamp();
                    DataLogManager.log("[Command] " + name + " started");
                })
                .finallyDo(interrupted -> DataLogManager.log(String.format(
                        "[Command] %s %s after %.2fs",
                        name, interrupted ? "interrupted" : "finished", Timer.getFPGATimestamp() - start[0])))
                .withName(name);
    }
}
