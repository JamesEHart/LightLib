# LightLogger
A logging library made by [Finn](https://github.com/JamesEHart) for FRC

LightLogger is a lightweight logging library designed for FRC teams. It provides a simple way to record robot states, sensor data, subsystem values, and poses during both real and simulation operations.

---

## Features

- High-speed logging via NetworkTables
- Log numbers, booleans, strings, events, warnings, errors, and geometry types
- Simple setup with one line in `Robot.java`
- Optional file logging to `.wpilog` files
- Live tuning: change numbers, booleans, and strings from a dashboard while the robot runs
- Frame timing utilities for loop performance tracking
- Compatible with NetworkTables dashboards (Glass, AdvantageScope, Elastic, etc.)

---

## Quick Start

### Installation
LightLogger is a WPILib vendor library for the **2026** season (Java only).

1. In VS Code, open the command palette (`Ctrl+Shift+P`) and run **WPILib: Manage Vendor Libraries**.
2. Choose **Install new library (online)**.
3. Paste this URL:
   ```
   https://raw.githubusercontent.com/JamesEHart/LightLogger/main/LightLogger.json
   ```
4. Build your project so Gradle downloads it.

### Import
```java
import io.github.jamesehart.lightlogger.LightLogger;
```

### Setup in your `Robot` constructor
```java
LightLogger.start(true);  // true = also save logs to a file, false = NetworkTables only
```

### Frame timing (optional, in `robotPeriodic()`)
```java
LightLogger.startFrame();
// ... your periodic code ...
LightLogger.endFrame();
```

---

## API Reference

### Lifecycle

| Method | Description |
|---|---|
| `LightLogger.start(boolean recordToFile)` | Starts the logger. Pass `true` to also record everything to a `.wpilog` file, `false` for NetworkTables only. |
| `LightLogger.stop()` | Flushes and stops the logger. |
| `LightLogger.startFrame()` | Records the start time of the current loop frame. |
| `LightLogger.endFrame()` | Flushes all pending NetworkTables updates. |
| `LightLogger.getFrameTime()` | Returns elapsed time in seconds since `startFrame()` was called. |

Log files are written using WPILib's `DataLogManager`, to a USB stick if one is plugged into the roboRIO, otherwise `/home/lvuser/logs`. In simulation they go to `logs/` in your project folder. Open them with AdvantageScope.

---

### Logging Primitives

All values are published to NetworkTables under the `/LightLogger/<key>` topic.

| Method | Description |
|---|---|
| `LightLogger.logNumber(String key, double value)` | Log a numeric value (doubles, ints, etc.). |
| `LightLogger.logBoolean(String key, boolean value)` | Log a boolean value. |
| `LightLogger.logString(String key, String value)` | Log a string value. |

**Example:**
```java
LightLogger.logNumber("Drive/LeftSpeed", leftMotor.get());
LightLogger.logBoolean("Intake/HasNote", intakeSensor.get());
LightLogger.logString("Robot/State", currentState.name());
```

---

### Events, Warnings, and Errors

These log timestamped messages (relative to the current frame) to fixed keys.

| Method | Published to | Description |
|---|---|---|
| `LightLogger.logEvent(String message)` | `/LightLogger/Events` | Log a general event. |
| `LightLogger.warn(String message)` | `/LightLogger/Warnings` | Log a warning. |
| `LightLogger.error(String message)` | `/LightLogger/Errors` | Log an error. |

**Example:**
```java
LightLogger.logEvent("AutoStarted");
LightLogger.warn("Vision target lost");
LightLogger.error("Motor controller disconnected");
```

---

### Geometry Logging

Geometry types are published as WPILib structs, making them compatible with AdvantageScope's 2D/3D field views.

| Method | Type |
|---|---|
| `LightLogger.logPose2d(String key, Pose2d pose)` | `Pose2d` |
| `LightLogger.logPose3d(String key, Pose3d pose)` | `Pose3d` |
| `LightLogger.logTranslation2d(String key, Translation2d t)` | `Translation2d` |
| `LightLogger.logTranslation3d(String key, Translation3d t)` | `Translation3d` |
| `LightLogger.logRotation2d(String key, Rotation2d r)` | `Rotation2d` |
| `LightLogger.logRotation3d(String key, Rotation3d r)` | `Rotation3d` |

**Example:**
```java
LightLogger.logPose2d("Drive/EstimatedPose", poseEstimator.getEstimatedPosition());
LightLogger.logRotation2d("Drive/Heading", gyro.getRotation2d());
```

---

### Live Tuning

Tunable values show up on your dashboard under `/LightLogger/<key>` like any other value, but you can also **edit** them while the robot is running, and the robot code uses the new value right away. Great for tuning PID gains, setpoints, and speeds without redeploying.

There are two ways to use them.

**1. Read the value every loop.** Call it in a periodic method and it returns the current value:

| Method | Description |
|---|---|
| `LightLogger.tunableNumber(String key, double defaultValue)` | Returns the current number. |
| `LightLogger.tunableBoolean(String key, boolean defaultValue)` | Returns the current boolean. |
| `LightLogger.tunableString(String key, String defaultValue)` | Returns the current string. |

```java
double speed = LightLogger.tunableNumber("Intake/Speed", 0.8);
intakeMotor.set(speed);
```

**2. Register once with a callback.** Call it once (e.g. in a constructor). The callback runs immediately with the starting value, then again every time the value is edited:

| Method | Description |
|---|---|
| `LightLogger.tunableNumber(String key, double defaultValue, Consumer<Double> onChange)` | Calls `onChange` when the number changes. |
| `LightLogger.tunableBoolean(String key, boolean defaultValue, Consumer<Boolean> onChange)` | Calls `onChange` when the boolean changes. |
| `LightLogger.tunableString(String key, String defaultValue, Consumer<String> onChange)` | Calls `onChange` when the string changes. |

```java
LightLogger.tunableNumber("Drive/kP", 0.1, pid::setP);
LightLogger.tunableNumber("Drive/kD", 0.0, pid::setD);
```

> Callbacks are checked in `LightLogger.startFrame()`, so you must call it at the top of `robotPeriodic()` for them to fire.

**Editing values from a dashboard:**
- **Glass:** open the NetworkTables view, find the value under `LightLogger`, and click it to edit.
- **AdvantageScope:** while connected live, turn on **Tuning Mode** (button at the top of the sidebar), then click a value in the sidebar to edit it.
- **Elastic:** add the value as a widget (e.g. Text Display or Toggle Switch) and edit it there.

**Things to know:**
- Edits are **not saved**. When robot code restarts, everything goes back to the defaults in your code, so copy tuned values back into your code when you're done.
- Don't use `logNumber` / `logBoolean` / `logString` with the same key as a tunable value, or the logged value will overwrite your edits.
- Edits are recorded in the `.wpilog` file along with everything else, so you can see what was changed and when.

---

## Development

### Repo layout

| Path | What it is |
|---|---|
| `src/main/java/` | The library itself. |
| `example/` | A robot project that uses LightLogger. It builds the library from this repo, so it's the place to test changes in simulation before releasing. |
| `LightLogger.json` | The vendordep file teams install. |
| `repos/` | The published Maven repository. GitHub serves it to GradleRIO. Generated by Gradle; don't edit by hand. |

### Testing locally
Open the `example/` folder in WPILib VS Code and run **Simulate Robot Code**.

### Releasing a new version
1. Bump `version` in `build.gradle`.
2. Bump both `version` fields in `LightLogger.json`, and copy the file to `example/vendordeps/`.
3. Run `./gradlew publish` from the repo root. The new version appears in `repos/`.
4. Commit and push to `main`. Teams get the update with **WPILib: Manage Vendor Libraries → Check for updates (online)**.

Use the WPILib JDK when running Gradle from a terminal (VS Code's WPILib terminal already does this):
```
JAVA_HOME=C:\Users\Public\wpilib\2026\jdk
```

### New season
Update `wpilibVersion` and the `wpilib/2026` path in `build.gradle`, and `frcYear` in `LightLogger.json`. Update the example project with the WPILib importer. Then release as above. GradleRIO refuses vendordeps whose `frcYear` doesn't match the project's year.
