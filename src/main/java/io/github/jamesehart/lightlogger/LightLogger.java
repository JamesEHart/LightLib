package io.github.jamesehart.lightlogger;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

/**
 * Lightweight logger that publishes values to NetworkTables under {@code /LightLogger/}, and
 * optionally records them to a WPILib data log file.
 */
public final class LightLogger {

    private static final String PREFIX = "/LightLogger/";
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private static final Map<String, DoublePublisher> numberPubs = new HashMap<>();
    private static final Map<String, BooleanPublisher> boolPubs = new HashMap<>();
    private static final Map<String, StringPublisher> stringPubs = new HashMap<>();

    private static final Map<String, StructPublisher<Pose2d>> pose2dStructPubs = new HashMap<>();
    private static final Map<String, StructPublisher<Pose3d>> pose3dStructPubs = new HashMap<>();
    private static final Map<String, StructPublisher<Translation2d>> translation2dStructPubs = new HashMap<>();
    private static final Map<String, StructPublisher<Translation3d>> translation3dStructPubs = new HashMap<>();
    private static final Map<String, StructPublisher<Rotation2d>> rotation2dStructPubs = new HashMap<>();
    private static final Map<String, StructPublisher<Rotation3d>> rotation3dStructPubs = new HashMap<>();

    private static boolean started = false;

    private static double frameStartTime = 0.0;

    private LightLogger() {}

    /** Records the start time of the current loop frame. Call at the top of robotPeriodic(). */
    public static void startFrame() {
        frameStartTime = Timer.getFPGATimestamp();
    }

    /** Flushes pending NetworkTables updates. Call at the bottom of robotPeriodic(). */
    public static void endFrame() {
        inst.flush();
    }

    /** Returns the seconds elapsed since {@link #startFrame()} was called. */
    public static double getFrameTime() {
        return Timer.getFPGATimestamp() - frameStartTime;
    }

    private static String frameTimestamped(String msg) {
        long us = (long) (getFrameTime() * 1_000_000);
        return "[" + us + "] " + msg;
    }

    /**
     * Starts the logger.
     *
     * @param recordToFile if true, also records all NetworkTables data to a .wpilog file (a USB
     *     stick if one is attached, otherwise /home/lvuser/logs, or ./logs in simulation)
     */
    public static void start(boolean recordToFile) {
        if (started) return;
        started = true;

        if (recordToFile) {
            DataLogManager.start();
            DataLogManager.logNetworkTables(true);
        }

        logEvent("LightLoggerStarted");
    }

    /** Logs a stop event and flushes pending NetworkTables updates. */
    public static void stop() {
        if (!started) return;
        logEvent("LightLoggerStopped");
        inst.flush();
        started = false;
    }

    /** Logs a numeric value to {@code /LightLogger/<key>}. */
    public static void logNumber(String key, double value) {
        DoublePublisher pub = numberPubs.computeIfAbsent(key,
                k -> inst.getDoubleTopic(PREFIX + k).publish());
        pub.set(value);
    }

    /** Logs a boolean value to {@code /LightLogger/<key>}. */
    public static void logBoolean(String key, boolean value) {
        BooleanPublisher pub = boolPubs.computeIfAbsent(key,
                k -> inst.getBooleanTopic(PREFIX + k).publish());
        pub.set(value);
    }

    /** Logs a string value to {@code /LightLogger/<key>}. */
    public static void logString(String key, String value) {
        StringPublisher pub = stringPubs.computeIfAbsent(key,
                k -> inst.getStringTopic(PREFIX + k).publish());
        pub.set(value);
    }

    /** Logs a frame-timestamped message to {@code /LightLogger/Events}. */
    public static void logEvent(String message) {
        logString("Events", frameTimestamped(message));
    }

    /** Logs a frame-timestamped message to {@code /LightLogger/Warnings}. */
    public static void warn(String message) {
        logString("Warnings", frameTimestamped(message));
    }

    /** Logs a frame-timestamped message to {@code /LightLogger/Errors}. */
    public static void error(String message) {
        logString("Errors", frameTimestamped(message));
    }

    private static StructPublisher<Pose2d> getPose2dStructPublisher(String key) {
        return pose2dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Pose2d.struct).publish());
    }

    private static StructPublisher<Pose3d> getPose3dStructPublisher(String key) {
        return pose3dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Pose3d.struct).publish());
    }

    private static StructPublisher<Translation2d> getTranslation2dStructPublisher(String key) {
        return translation2dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Translation2d.struct).publish());
    }

    private static StructPublisher<Translation3d> getTranslation3dStructPublisher(String key) {
        return translation3dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Translation3d.struct).publish());
    }

    private static StructPublisher<Rotation2d> getRotation2dStructPublisher(String key) {
        return rotation2dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Rotation2d.struct).publish());
    }

    private static StructPublisher<Rotation3d> getRotation3dStructPublisher(String key) {
        return rotation3dStructPubs.computeIfAbsent(key,
                k -> inst.getStructTopic(PREFIX + k, Rotation3d.struct).publish());
    }

    /** Logs a {@link Pose2d} as a struct to {@code /LightLogger/<key>}. */
    public static void logPose2d(String key, Pose2d pose) {
        getPose2dStructPublisher(key).set(pose);
    }

    /** Logs a {@link Pose3d} as a struct to {@code /LightLogger/<key>}. */
    public static void logPose3d(String key, Pose3d pose) {
        getPose3dStructPublisher(key).set(pose);
    }

    /** Logs a {@link Translation2d} as a struct to {@code /LightLogger/<key>}. */
    public static void logTranslation2d(String key, Translation2d t) {
        getTranslation2dStructPublisher(key).set(t);
    }

    /** Logs a {@link Translation3d} as a struct to {@code /LightLogger/<key>}. */
    public static void logTranslation3d(String key, Translation3d t) {
        getTranslation3dStructPublisher(key).set(t);
    }

    /** Logs a {@link Rotation2d} as a struct to {@code /LightLogger/<key>}. */
    public static void logRotation2d(String key, Rotation2d r) {
        getRotation2dStructPublisher(key).set(r);
    }

    /** Logs a {@link Rotation3d} as a struct to {@code /LightLogger/<key>}. */
    public static void logRotation3d(String key, Rotation3d r) {
        getRotation3dStructPublisher(key).set(r);
    }
}
