package io.github.jamesehart.lightutil.mechanisms;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Commands that find a mechanism's zero by gently driving it into a hard stop or limit switch,
 * then resetting its encoder. Good for elevators and arms with relative encoders.
 *
 * <pre>{@code
 * Command home = Homing.toHardStop(
 *     elevator::setVoltage, () -> motor.getStatorCurrent().getValueAsDouble(),
 *     -1.5, 20, () -> motor.setPosition(0), 3.0, elevator);
 * }</pre>
 *
 * <p>If the timeout is reached first, the encoder is <b>not</b> zeroed and a warning is printed.
 */
public final class Homing {
    private static final double STARTUP_IGNORE_SECONDS = 0.25;

    private Homing() {}

    /**
     * Drives into a hard stop until the current rises, then zeroes.
     *
     * @param voltage sets the motor voltage
     * @param current motor current, amps (sign ignored)
     * @param homingVolts voltage to drive with, toward the hard stop (small, e.g. -1.5)
     * @param stallAmps current that means "hit the stop"
     * @param zero resets the encoder, e.g. {@code () -> motor.setPosition(0)}
     * @param timeoutSeconds give up after this long
     * @param mechanism the mechanism's subsystem
     * @return the homing command
     */
    public static Command toHardStop(
            DoubleConsumer voltage,
            DoubleSupplier current,
            double homingVolts,
            double stallAmps,
            Runnable zero,
            double timeoutSeconds,
            Subsystem mechanism) {
        // Ignore the current spike as the motor starts moving.
        double[] start = new double[1];
        return home(
                voltage,
                () -> Timer.getFPGATimestamp() - start[0] > STARTUP_IGNORE_SECONDS
                        && Math.abs(current.getAsDouble()) > stallAmps,
                () -> start[0] = Timer.getFPGATimestamp(),
                homingVolts,
                zero,
                timeoutSeconds,
                mechanism,
                "HomeToHardStop");
    }

    /**
     * Drives toward a limit switch until it's pressed, then zeroes.
     *
     * @param voltage sets the motor voltage
     * @param limitSwitch true when the switch is pressed
     * @param homingVolts voltage to drive with, toward the switch
     * @param zero resets the encoder
     * @param timeoutSeconds give up after this long
     * @param mechanism the mechanism's subsystem
     * @return the homing command
     */
    public static Command toLimitSwitch(
            DoubleConsumer voltage,
            BooleanSupplier limitSwitch,
            double homingVolts,
            Runnable zero,
            double timeoutSeconds,
            Subsystem mechanism) {
        return home(voltage, limitSwitch, () -> {}, homingVolts, zero, timeoutSeconds, mechanism, "HomeToLimitSwitch");
    }

    private static Command home(
            DoubleConsumer voltage,
            BooleanSupplier atZero,
            Runnable onStart,
            double homingVolts,
            Runnable zero,
            double timeoutSeconds,
            Subsystem mechanism,
            String name) {
        Debouncer debouncer = new Debouncer(0.1);
        double[] start = new double[1];
        boolean[] homed = new boolean[1];
        return new FunctionalCommand(
                        () -> {
                            start[0] = Timer.getFPGATimestamp();
                            homed[0] = false;
                            debouncer.calculate(false);
                            onStart.run();
                        },
                        () -> voltage.accept(homingVolts),
                        interrupted -> {
                            voltage.accept(0);
                            if (homed[0]) {
                                zero.run();
                            } else if (!interrupted) {
                                DriverStation.reportWarning(name + " timed out; encoder not zeroed", false);
                            }
                        },
                        () -> {
                            homed[0] = debouncer.calculate(atZero.getAsBoolean());
                            return homed[0] || Timer.getFPGATimestamp() - start[0] > timeoutSeconds;
                        },
                        mechanism)
                .withName(name);
    }
}
