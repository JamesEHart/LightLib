package io.github.jamesehart.lightutil.diagnostics;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Watches your devices and shows problems as WPILib {@link Alert}s, which appear in Elastic,
 * Shuffleboard and AdvantageScope. Works with any vendor: you pass a lambda.
 *
 * <pre>{@code
 * HealthMonitor.connected("FL Drive", () -> flDrive.isConnected());
 * HealthMonitor.connected("Intake", () -> !intakeSpark.getFaults().can);
 * HealthMonitor.temperature("Shooter", () -> shooter.getDeviceTemp().getValueAsDouble(), 70);
 * HealthMonitor.lowBattery(12.3);
 * }</pre>
 *
 * <p>A problem shows once it has lasted half a second, so a single dropped CAN frame doesn't flag.
 * Disconnects are also remembered: after a device comes back, a warning ("was disconnected 3
 * times") stays up so you can see it after the match.
 *
 * <p>Checks run automatically every loop through the command scheduler; nothing to call from
 * periodic.
 */
public final class HealthMonitor {
    private static final double DEBOUNCE_SECONDS = 0.5;

    private HealthMonitor() {}

    /**
     * Shows an error while a device is disconnected, and a lasting warning after it reconnects.
     *
     * @param name device name, e.g. "FL Drive"
     * @param isConnected true while the device is talking on the bus
     */
    public static void connected(String name, BooleanSupplier isConnected) {
        Alert now = new Alert(name + " disconnected", AlertType.kError);
        Alert history = new Alert(name + " was disconnected", AlertType.kWarning);
        Debouncer debouncer = new Debouncer(DEBOUNCE_SECONDS, DebounceType.kRising);
        int[] count = new int[1];
        poll(() -> {
            boolean down = debouncer.calculate(!isConnected.getAsBoolean());
            if (down && !now.get()) {
                count[0]++;
                history.setText(name + " was disconnected " + count[0] + (count[0] == 1 ? " time" : " times"));
            }
            now.set(down);
            history.set(count[0] > 0 && !down);
        });
    }

    /**
     * Shows a warning while a device is hotter than {@code maxCelsius}.
     *
     * @param name device name
     * @param celsius the device's temperature
     * @param maxCelsius warning threshold, e.g. 70 for most brushless motors
     */
    public static void temperature(String name, DoubleSupplier celsius, double maxCelsius) {
        Alert alert = new Alert(name + " hot", AlertType.kWarning);
        Debouncer debouncer = new Debouncer(DEBOUNCE_SECONDS, DebounceType.kRising);
        poll(() -> {
            double temp = celsius.getAsDouble();
            boolean hot = debouncer.calculate(temp > maxCelsius);
            if (hot) alert.setText(String.format("%s hot (%.0f°C)", name, temp));
            alert.set(hot);
        });
    }

    /**
     * Shows an alert while a condition is false. For anything else worth watching, e.g. a sensor
     * reading out of range or a mechanism not homed.
     *
     * @param message what to show when it's false, e.g. "Elevator not homed"
     * @param isOk true when everything is fine
     * @param type how serious it is
     */
    public static void check(String message, BooleanSupplier isOk, AlertType type) {
        Alert alert = new Alert(message, type);
        Debouncer debouncer = new Debouncer(DEBOUNCE_SECONDS, DebounceType.kRising);
        poll(() -> alert.set(debouncer.calculate(!isOk.getAsBoolean())));
    }

    /**
     * Warns while disabled if the battery is below {@code minVolts}, so you swap it before the
     * match. Only checks while disabled, since voltage always drops while driving.
     *
     * @param minVolts lowest acceptable resting voltage, e.g. 12.3
     */
    public static void lowBattery(double minVolts) {
        Alert alert = new Alert("Low battery", AlertType.kWarning);
        Debouncer debouncer = new Debouncer(2.0, DebounceType.kRising);
        poll(() -> {
            double volts = RobotController.getBatteryVoltage();
            boolean low = debouncer.calculate(DriverStation.isDisabled() && volts < minVolts);
            if (low) alert.setText(String.format("Low battery (%.2f V), swap it before the match", volts));
            alert.set(low);
        });
    }

    /**
     * Warns while CAN bus utilization is above {@code maxFraction}. A busy bus drops frames, which
     * looks like random disconnects.
     *
     * @param maxFraction 0 to 1, e.g. 0.9
     */
    public static void canUtilization(double maxFraction) {
        Alert alert = new Alert("CAN bus busy", AlertType.kWarning);
        Debouncer debouncer = new Debouncer(1.0, DebounceType.kRising);
        poll(() -> {
            double used = RobotController.getCANStatus().percentBusUtilization;
            boolean busy = debouncer.calculate(used > maxFraction);
            if (busy) alert.setText(String.format("CAN bus %.0f%% busy", used * 100));
            alert.set(busy);
        });
    }

    private static void poll(Runnable check) {
        CommandScheduler.getInstance().getDefaultButtonLoop().bind(check);
    }
}
