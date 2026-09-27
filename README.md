# LightLib
Lightweight FRC libraries made by [Finn](https://github.com/JamesEHart), for the **2026** season (Java only).

| Library | What it does | Vendordep URL |
|---|---|---|
| [LightLogger](#lightlogger) | Logging and live tuning over NetworkTables | `https://raw.githubusercontent.com/JamesEHart/LightLib/main/LightLogger.json` |
| [LightSim](#lightsim) | Physics simulation for motors, mechanisms, and drivetrains, viewed in AdvantageScope | `https://raw.githubusercontent.com/JamesEHart/LightLib/main/LightSim.json` |

Install either one, or both, from VS Code: open the command palette (`Ctrl+Shift+P`), run **WPILib: Manage Vendor Libraries → Install new library (online)**, and paste the URL.

---

## LightLogger

LightLogger is a lightweight logging library designed for FRC teams. It provides a simple way to record robot states, sensor data, subsystem values, and poses during both real and simulation operations.

### Features

- High-speed logging via NetworkTables
- Log numbers, booleans, strings, events, warnings, errors, and geometry types
- Simple setup with one line in `Robot.java`
- Optional file logging to `.wpilog` files
- Live tuning: change numbers, booleans, and strings from a dashboard while the robot runs
- Frame timing utilities for loop performance tracking
- Compatible with NetworkTables dashboards (Glass, AdvantageScope, Elastic, etc.)

---

### Quick Start

#### Installation
1. In VS Code, open the command palette (`Ctrl+Shift+P`) and run **WPILib: Manage Vendor Libraries**.
2. Choose **Install new library (online)**.
3. Paste this URL:
   ```
   https://raw.githubusercontent.com/JamesEHart/LightLib/main/LightLogger.json
   ```
4. Build your project so Gradle downloads it.

#### Import
```java
import io.github.jamesehart.lightlogger.LightLogger;
```

#### Setup in your `Robot` constructor
```java
LightLogger.start(true);  // true = also save logs to a file, false = NetworkTables only
```

#### Frame timing (optional, in `robotPeriodic()`)
```java
LightLogger.startFrame();
// ... your periodic code ...
LightLogger.endFrame();
```

---

### API Reference

#### Lifecycle

| Method | Description |
|---|---|
| `LightLogger.start(boolean recordToFile)` | Starts the logger. Pass `true` to also record everything to a `.wpilog` file, `false` for NetworkTables only. |
| `LightLogger.stop()` | Flushes and stops the logger. |
| `LightLogger.startFrame()` | Records the start time of the current loop frame. |
| `LightLogger.endFrame()` | Flushes all pending NetworkTables updates. |
| `LightLogger.getFrameTime()` | Returns elapsed time in seconds since `startFrame()` was called. |

Log files are written using WPILib's `DataLogManager`, to a USB stick if one is plugged into the roboRIO, otherwise `/home/lvuser/logs`. In simulation they go to `logs/` in your project folder. Open them with AdvantageScope.

---

#### Logging Primitives

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

#### Events, Warnings, and Errors

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

#### Geometry Logging

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

#### Live Tuning

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

## LightSim

LightSim simulates your robot with real physics, with as little setup as possible:

- Swap your motor controllers for LightSim's versions (`TalonFX` → `SimTalonFX`).
- Add one line to your `Robot` constructor.
- Motors spin with real motor physics, mechanisms feel gravity and hit their limits, and your drivetrain pushes a rigid body around the field, colliding with the walls (using the [dyn4j](https://dyn4j.org) physics engine).
- Everything shows up in **AdvantageScope** on its own 2026 field and robot models.

On a real robot the `Sim*` classes are exactly the normal vendor classes and LightSim does nothing, so you never have to remove it.

### Features
- TalonFX, TalonFXS, TalonSRX, SparkMax, and SparkFlex, including their onboard closed-loop control (PID, Motion Magic, MAXMotion) and your inversion and gear-ratio configs
- Flywheels/rollers, elevators, and arms with gravity and hard stops
- Swerve and tank drivetrains with wheel traction limits (wheels slip if you push too hard or drive into a wall)
- Simulated Pigeon2 that follows the robot's heading
- Battery voltage sag when motors draw current
- Field perimeter walls and custom obstacles

### Quick Start

#### Installation
1. Install LightSim with the URL above.
2. Also install the vendor libraries for the motor controllers you use (Phoenix 6, Phoenix 5 and/or REVLib), as you normally would. LightSim doesn't install them for you, so REV-only teams don't need Phoenix and vice versa.

#### Turn it on
At the very top of your `Robot` constructor:
```java
import io.github.jamesehart.lightsim.LightSim;

LightSim.enable(this);
```

#### Swap your motor controllers
| Instead of | Use | Default motor |
|---|---|---|
| `new TalonFX(id)` | `new SimTalonFX(id)` | Kraken X60 |
| `new TalonFXS(id)` | `new SimTalonFXS(id, DCMotor.getMinion(1))` | none, you pass it |
| `new WPI_TalonSRX(id)` | `new SimTalonSRX(id, DCMotor.getCIM(1))` | none, you pass it |
| `new SparkMax(id, type)` | `new SimSparkMax(id, type)` | NEO (brushless) / CIM (brushed) |
| `new SparkFlex(id, type)` | `new SimSparkFlex(id, type)` | NEO Vortex (brushless) / CIM (brushed) |
| `new Pigeon2(id)` | `new SimPigeon2(id)` | |

They're in `io.github.jamesehart.lightsim.motors`. Every constructor also takes a `DCMotor` to use a different motor, e.g. `new SimTalonFX(1, DCMotor.getFalcon500(1))`, and the CTRE ones take a `CANBus` like normal. Since they extend the vendor classes, all your existing code and configs keep working.

That's already enough: every motor now spins a light load, so encoders and closed-loop control respond. To make the physics match your robot, tell LightSim what each motor is connected to.

#### Mechanisms
Call these once, after creating the motors (e.g. in a subsystem constructor):

```java
// Flywheel/roller/intake: gearing, moment of inertia (kg·m²)
LightSim.flywheel(shooterMotor, 1.0, 0.004);

// Elevator: gearing, carriage mass (kg), drum radius (m), min and max height (m)
LightSim.elevator(elevatorMotor, 10, 8, 0.03, 0, 1.2);

// Arm: gearing, moment of inertia (kg·m²), length (m), min and max angle (rad; 0 = horizontal)
LightSim.arm(armMotor, 60, 0.5, 0.6, Math.toRadians(-30), Math.toRadians(110));
```

For several motors on one mechanism, pass an array: `LightSim.flywheel(new SimMotor[] {left, right}, 1.0, 0.004)`. Each returns the mechanism, so you can read `getPosition()` / `getVelocity()`.

#### Swerve drivetrain
```java
import io.github.jamesehart.lightsim.physics.SwerveSimConfig;

LightSim.swerve(new SwerveSimConfig()
    .module(flDrive, flSteer, new Translation2d( 0.28,  0.28))   // x forward, y left, meters
    .module(frDrive, frSteer, new Translation2d( 0.28, -0.28))
    .module(blDrive, blSteer, new Translation2d(-0.28,  0.28))
    .module(brDrive, brSteer, new Translation2d(-0.28, -0.28))
    .driveGearing(6.75)
    .steerGearing(150.0 / 7)
    .wheelRadius(0.0508)
    .robotMass(55)                // kg, with bumpers and battery
    .bumperSize(0.9, 0.9)         // meters
    .gyro(pigeon));               // optional SimPigeon2

LightSim.setRobotPose(new Pose2d(2, 4, Rotation2d.kZero));  // starting position
```

#### Tank drivetrain
```java
import io.github.jamesehart.lightsim.physics.TankSimConfig;

LightSim.tank(new TankSimConfig()
    .left(leftLeader, leftFollower)
    .right(rightLeader, rightFollower)
    .trackWidth(0.6)
    .gearing(8.45)
    .wheelRadius(0.0762)
    .robotMass(50)
    .gyro(pigeon));
```

Configure your motors (inversion etc.) exactly like on your real robot. LightSim assumes a positive command drives the robot forward and turns swerve modules counter-clockwise.

#### Field
The field walls match the official 2026 field. Add other things the robot should bump into as boxes:
```java
LightSim.addObstacle(new Pose2d(4.6, 4.0, Rotation2d.kZero), 1.2, 1.2);  // center, length, width
```

`LightSim.getRobotPose()` returns the robot's true simulated pose, which is handy for checking your odometry.

### Viewing it in AdvantageScope
LightSim publishes everything under `/LightSim/` in the formats AdvantageScope understands:

| Topic | Type | Use it for |
|---|---|---|
| `/LightSim/RobotPose` | `Pose2d` | The robot on the 2D or 3D field |
| `/LightSim/RobotPose3d` | `Pose3d` | Same, as a 3D pose |
| `/LightSim/SwerveStates` | `SwerveModuleState[]` | The Swerve tab |
| `/LightSim/Components` | `Pose3d[]` | Moving parts of your robot model (elevators, arms) |
| `/LightSim/BatteryVoltage` | `double` | Battery sag |

1. Run **Simulate Robot Code**, then in AdvantageScope choose **File → Connect to Simulator**.
2. Open a **3D Field** tab and pick the **2026** field.
3. Drag `/LightSim/RobotPose` onto the tab's pose list and pick a robot model: one of AdvantageScope's built-in robots, or your own.
4. For swerve, open a **Swerve** tab and drag in `/LightSim/SwerveStates`.

**Moving parts:** if your robot model has components (the `components` list in its `config.json`; see AdvantageScope's docs on custom robot models), tell LightSim which mechanism is which component:
```java
LightSim.elevator(elevatorMotor, 10, 8, 0.03, 0, 1.2)
    .component(0, new Translation3d(0, 0, 0.1));          // component 0, moves up along z
LightSim.arm(armMotor, 60, 0.5, 0.6, -0.5, 1.9)
    .component(1, new Translation3d(0.2, 0, 0.5));        // component 1, pivots here
```
Then drag `/LightSim/Components` onto the robot in the 3D Field tab as its component poses. By default, elevators slide along +z and arms pivot about -y. Pass a third argument (an axis) to change that, e.g. `new Translation3d(0, 0, 1)` for a turret.

### Limitations
- No game pieces, field elements (other than walls and your obstacles), other robots, or vision yet.
- Brake mode is assumed: a motor with no output resists being turned.
- Swerve steering, drive wheels, and the robot body are simplified models. They're good for testing code and tuning, not for predicting exact cycle times.


---

## Development

### Repo layout

| Path | What it is |
|---|---|
| `lightlogger/` | The LightLogger library. |
| `lightsim/` | The LightSim library. `physics/` is pure Java with unit tests; `motors/` wraps the vendor classes. |
| `example/` | A robot project using both libraries (swerve, flywheel, elevator). It builds the libraries from this repo, so it's the place to test changes in simulation before releasing. |
| `LightLogger.json`, `LightSim.json` | The vendordep files teams install. |
| `repos/` | The published Maven repository. GitHub serves it to GradleRIO. Generated by Gradle; don't edit by hand. |

### Testing locally
- `./gradlew :lightsim:test` runs the physics tests.
- Open the `example/` folder in WPILib VS Code and run **Simulate Robot Code**, then connect AdvantageScope. `./gradlew simulateJava -Pheadless` in `example/` runs it without the sim GUI.

### Releasing a new version
1. Bump `version` in `lightlogger/build.gradle` or `lightsim/build.gradle`.
2. Bump the matching versions in `LightLogger.json` / `LightSim.json`, and copy the file to `example/vendordeps/`.
3. Run `./gradlew publish` from the repo root. The new version appears in `repos/`.
4. Commit and push to `main`. Teams get the update with **WPILib: Manage Vendor Libraries → Check for updates (online)**.

Use the WPILib JDK when running Gradle from a terminal (VS Code's WPILib terminal already does this):
```
JAVA_HOME=C:\Users\Public\wpilib\2026\jdk
```

### New season
- Update `wpilibVersion` and the `wpilib/2026` path in the root `build.gradle`, and the Phoenix/REVLib versions in `lightsim/build.gradle`.
- Update `frcYear` in both vendordep JSONs.
- Update the example project with the WPILib importer.
- Then release as above. GradleRIO refuses vendordeps whose `frcYear` doesn't match the project's year.
