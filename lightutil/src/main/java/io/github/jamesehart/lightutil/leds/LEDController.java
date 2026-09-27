package io.github.jamesehart.lightutil.leds;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An addressable LED strip where several things can ask for a pattern at once, and the most
 * important one wins. For example, "has game piece" (priority 1) shows over the idle alliance color
 * (the default), and "ready to shoot" (priority 2) shows over both. When a request ends, the next
 * one down shows again.
 *
 * <p>Patterns are WPILib {@link LEDPattern}s, so all of its effects (blink, breathe, rainbow,
 * progress bar, scroll) work.
 *
 * <pre>{@code
 * LEDController leds = new LEDController(0, 60);   // PWM port, LED count
 * leds.setDefault(LEDPattern.solid(Color.kBlue));
 * hasPiece.whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 1));
 * readyToShoot.whileTrue(leds.show(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1)), 2));
 * }</pre>
 *
 * <p>It's a subsystem, so it updates itself every loop once created. The {@link #show} commands
 * don't require it, so they never interrupt each other.
 */
public final class LEDController extends SubsystemBase {
    private record Layer(int priority, long order, LEDPattern pattern) {}

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final Map<String, Layer> layers = new HashMap<>();
    private LEDPattern defaultPattern = LEDPattern.kOff;
    private long nextOrder = 0;
    private int nextAnonymous = 0;

    /**
     * Creates the controller and starts the strip.
     *
     * @param pwmPort roboRIO PWM port the strip is plugged into
     * @param length number of LEDs
     */
    public LEDController(int pwmPort, int length) {
        led = new AddressableLED(pwmPort);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.start();
    }

    /** The pattern shown when nothing else is requested. Defaults to off. */
    public void setDefault(LEDPattern pattern) {
        defaultPattern = pattern;
    }

    /**
     * Shows a pattern until {@link #clear(String)} is called with the same name. Calling again with
     * the same name replaces it.
     *
     * @param name identifies this request
     * @param priority higher shows on top; ties go to the newest request
     * @param pattern what to show
     */
    public void set(String name, int priority, LEDPattern pattern) {
        layers.put(name, new Layer(priority, nextOrder++, pattern));
    }

    /** Removes a request made with {@link #set}. */
    public void clear(String name) {
        layers.remove(name);
    }

    /**
     * A command that shows a pattern while it runs, e.g. {@code trigger.whileTrue(leds.show(...))}.
     *
     * @param pattern what to show
     * @param priority higher shows on top
     * @return the command
     */
    public Command show(LEDPattern pattern, int priority) {
        String name = "show" + nextAnonymous++;
        return Commands.startEnd(() -> set(name, priority, pattern), () -> clear(name))
                .ignoringDisable(true)
                .withName("LEDs/" + name);
    }

    /** The pattern currently being shown. */
    public LEDPattern getActivePattern() {
        Layer top = null;
        for (Layer layer : layers.values()) {
            if (top == null
                    || layer.priority() > top.priority()
                    || (layer.priority() == top.priority() && layer.order() > top.order())) {
                top = layer;
            }
        }
        return top == null ? defaultPattern : top.pattern();
    }

    @Override
    public void periodic() {
        getActivePattern().applyTo(buffer);
        led.setData(buffer);
    }
}
