package io.github.jamesehart.lightutil.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Picks a config based on which roboRIO the code is running on, so one codebase can run on your
 * competition robot and practice robot with different encoder offsets, gear ratios, etc.
 *
 * <pre>{@code
 * record Constants(double armOffset) {}
 *
 * Constants constants = new RobotIdentity<>(new Constants(0.12))   // default: competition robot
 *     .add("0316B37A", new Constants(0.47))                        // practice robot's serial number
 *     .get();
 * }</pre>
 *
 * <p>Find a roboRIO's serial number on the sticker, in the roboRIO web dashboard, or from
 * {@link #serialNumber()} (it's printed when {@link #get()} runs).
 *
 * @param <T> the config type
 */
public final class RobotIdentity<T> {
    private final T defaultConfig;
    private final Map<String, T> bySerial = new HashMap<>();
    private T simConfig;

    /**
     * @param defaultConfig used on any roboRIO not added with {@link #add}
     */
    public RobotIdentity(T defaultConfig) {
        this.defaultConfig = defaultConfig;
    }

    /**
     * Uses {@code config} on the roboRIO with this serial number.
     *
     * @param serialNumber the roboRIO serial number, e.g. "0316B37A"
     * @param config its config
     * @return this, for chaining
     */
    public RobotIdentity<T> add(String serialNumber, T config) {
        bySerial.put(serialNumber.toUpperCase(), config);
        return this;
    }

    /** Uses {@code config} in simulation. Otherwise the default is used. */
    public RobotIdentity<T> sim(T config) {
        this.simConfig = config;
        return this;
    }

    /** The config for the roboRIO this is running on. */
    public T get() {
        if (RobotBase.isSimulation()) return simConfig != null ? simConfig : defaultConfig;
        String serial = serialNumber();
        T config = bySerial.get(serial);
        if (config == null) {
            DriverStation.reportWarning("[RobotIdentity] Unknown roboRIO " + serial + ", using default config", false);
            return defaultConfig;
        }
        System.out.println("[RobotIdentity] roboRIO " + serial);
        return config;
    }

    /** This roboRIO's serial number, uppercase. */
    public static String serialNumber() {
        return RobotController.getSerialNumber().toUpperCase();
    }
}
