package io.github.jamesehart.lightutil.mechanisms;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Detects a motor stalling: high current while barely moving. Useful for "the intake has a game
 * piece" (the rollers stall on it) and for finding a hard stop.
 *
 * <pre>{@code
 * Trigger hasPiece = StallDetector.trigger(
 *     () -> intakeMotor.getStatorCurrent().getValueAsDouble(), 30,
 *     () -> intakeMotor.getVelocity().getValueAsDouble(), 2,
 *     0.1);
 * hasPiece.onTrue(Rumble.rumble(controller, 0.8, 0.3));
 * }</pre>
 */
public final class StallDetector {
    private StallDetector() {}

    /**
     * True once current has been above {@code amps} and speed below {@code maxVelocity} for {@code
     * seconds}.
     *
     * @param current motor current, amps (sign ignored)
     * @param amps current threshold
     * @param velocity motor or mechanism velocity, any unit (sign ignored)
     * @param maxVelocity velocity below which the motor counts as stopped, same unit
     * @param seconds how long it must stall; filters out current spikes when the motor starts
     * @return the stall trigger
     */
    public static Trigger trigger(
            DoubleSupplier current, double amps, DoubleSupplier velocity, double maxVelocity, double seconds) {
        return new Trigger(() -> Math.abs(current.getAsDouble()) > amps
                        && Math.abs(velocity.getAsDouble()) < maxVelocity)
                .debounce(seconds, DebounceType.kRising);
    }

    /**
     * True once current has been above {@code amps} for {@code seconds}. For when you don't have
     * a velocity measurement.
     *
     * @param current motor current, amps (sign ignored)
     * @param amps current threshold
     * @param seconds how long it must stay high
     * @return the stall trigger
     */
    public static Trigger trigger(DoubleSupplier current, double amps, double seconds) {
        return new Trigger(() -> Math.abs(current.getAsDouble()) > amps).debounce(seconds, DebounceType.kRising);
    }
}
