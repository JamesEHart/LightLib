# LightUtil

LightUtil is a toolbox of the helpers most robots end up writing from scratch: aiming a shooter or turret, flipping field positions for the red alliance, shaping joystick input, driving to a pose, filtering vision, pre-match checks, and more.

- **Works with any hardware.** It only uses WPILib types (`Pose2d`, `ChassisSpeeds`, `Trigger`, `LEDPattern`) and lambdas, so it doesn't care whether you use CTRE, REV, PhotonVision or Limelight.
- **Pick what you need.** Every class stands on its own; there's no framework to set up.
- Doesn't depend on LightLogger or LightSim, but works well with both.

## Installation
In VS Code, open the command palette (`Ctrl+Shift+P`), run **WPILib: Manage Vendor Libraries → Install new library (online)**, and paste:
```
https://raw.githubusercontent.com/JamesEHart/LightLib/main/LightUtil.json
```
It uses WPILib's command-based library (`WPILibNewCommands`), which every command-based project already has.

A few classes check things every loop (`HealthMonitor`, `AutoStartChecker`, `LEDController`). They run from `CommandScheduler.getInstance().run()`, which command-based robots already call in `robotPeriodic()`. If yours is a plain `TimedRobot`, add that call.

## Packages
Everything is under `io.github.jamesehart.lightutil`. Units are meters, radians and seconds.

| Package | Classes | What for |
|---|---|---|
| [`math`](#math) | `AngleUtil`, `GeomUtil`, `InterpolatingTable`, `Polygon2d` | Angle wrapping and turret range placement, pose helpers, lookup tables, polygons |
| [`field`](#field) | `AllianceFlip`, `FieldZone` | Write positions once for blue and use them on red; "robot is in this area" triggers |
| [`targeting`](#targeting) | `Target`, `AimSolver`, `ShotTable`, `ShootOnTheMove` | Distance and angle to a goal, turret aiming, shooter tables, aiming while driving |
| [`drive`](#drive) | `DriveInput`, `PoseController`, `SkidDetector`, `WheelRadiusCharacterization` | Joysticks → speeds with aim assist, auto-align to a pose, wheel slip detection, measuring wheel radius |
| [`auto`](#auto) | `AutoStartChecker` | Warns when the robot isn't placed where the selected auto starts |
| [`input`](#input) | `InputCurves`, `ButtonPatterns`, `Rumble` | Deadband and curves, double tap / tap vs. hold / chords, controller rumble |
| [`commands`](#commands) | `StateMachine`, `TriggerUtil`, `CommandUtil` | Enum state machines, latches and toggles, timeouts and command logging |
| [`mechanisms`](#mechanisms) | `StallDetector`, `Homing`, `SysIdHelper` | "Has game piece" detection, zeroing against a hard stop, one-line SysId setup |
| [`leds`](#leds) | `LEDController` | An LED strip where the most important request wins |
| [`vision`](#vision) | `VisionFilter`, `ObjectDetection`, `GamePieceTracker` | Reject bad AprilTag estimates and weight good ones, find and remember game pieces |
| [`diagnostics`](#diagnostics) | `HealthMonitor` | Dashboard alerts for disconnected or hot devices, low battery, busy CAN bus |
| [`robot`](#robot) | `MatchTimer`, `RobotIdentity` | Endgame warnings, different constants for your comp and practice robots |
| [`season2026`](#season2026) | `HubShift` | REBUILT only: whether your hub is active |

Everything is also documented in the Javadoc, which VS Code shows when you hover over a class or method.

---

## math
```java
AngleUtil.wrapPi(angle);                         // into [-π, π)
AngleUtil.shortestDelta(from, to);               // shortest signed turn
AngleUtil.placeInRange(target, current, min, max);  // equivalent angle inside a limited range

GeomUtil.nearest(pose, scoringPoses);            // closest pose in a list
GeomUtil.extrapolate(pose, fieldSpeeds, 0.1);    // where the robot will be in 0.1 s
GeomUtil.isNear(a, b, 0.05, Math.toRadians(3));

InterpolatingTable table = new InterpolatingTable()
    .add(2.0, 3000, 0.40)    // input -> any number of outputs
    .add(4.0, 3800, 0.60);
double[] out = table.get(3.0);                   // {3400, 0.50}; clamped outside the table

Polygon2d zone = Polygon2d.rectangle(new Translation2d(0, 0), new Translation2d(4, 8.07));
zone.contains(pose.getTranslation());
```

## field
Write every field position from the **blue** alliance's side (WPILib's origin is the blue corner) and flip it when you use it:
```java
Pose2d start = AllianceFlip.ifRed(new Pose2d(3.5, 2.0, Rotation2d.kZero));
```
The 2026 field is rotated 180° between alliances, which is the default. For a mirrored field (like 2024), call `AllianceFlip.setSymmetry(Symmetry.MIRROR)`.

`FieldZone` turns an area of the field into a trigger, flipped for red automatically:
```java
FieldZone ourHalf = new FieldZone("OurHalf", Polygon2d.rectangle(
        new Translation2d(0, 0), new Translation2d(8.27, 8.07)));
ourHalf.trigger(drive::getPose).onFalse(Commands.print("Crossed the center line"));
```

## targeting
```java
Target goal = Target.of("Goal", new Translation3d(4.6, 4.0, 1.8));   // blue position; flips on red

ShotTable table = new ShotTable()           // distance (m) → rpm, hood angle (rad), time of flight (s)
    .add(1.5, 2800, Math.toRadians(20), 0.45)
    .add(3.0, 3400, Math.toRadians(32), 0.70)
    .add(5.0, 4300, Math.toRadians(41), 1.05);

// Standing still:
double distance = AimSolver.distance(pose, goal.get2d());
Rotation2d faceGoal = AimSolver.fieldAngle(pose, goal.get2d());
ShotTable.Shot shot = table.get(distance);

// While driving: aims at a shifted "virtual target" so the piece's sideways speed carries it in.
ShootOnTheMove.Solution s = ShootOnTheMove.solve(pose, fieldSpeeds, goal.get2d(), table);
```
**Turrets:** `AimSolver.turretAngle(pose, robotToTurret, target, currentAngle, minAngle, maxAngle)` gives the turret angle within its legal range, taking the shortest path without winding past a cable limit. Add `AimSolver.turretVelocityFeedforward(...)` to your turret feedforward so it keeps up while the robot turns.

For latency compensation, pass a predicted pose (`GeomUtil.extrapolate`) instead of the current one.

## drive
`DriveInput` turns joysticks into drivetrain speeds, with a deadband, a squared response curve, and an optional acceleration limit and slow mode. Field-relative driving is alliance-aware: stick forward always drives away from your driver station.
```java
DriveInput input = new DriveInput(
        () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX())
    .maxSpeed(4.5)
    .maxTurnRate(2 * Math.PI)
    .accelerationLimit(8, 20)
    .slowMode(controller::getLeftBumperButton, 0.4);

drive.driveRobotRelative(input.fieldRelative(drive.getPose().getRotation()));
```
`speedScale(() -> 1 - 0.7 * controller.getLeftTriggerAxis())` gives analog slow-down on a trigger.

**Aim assist:** while the heading override returns a heading, the robot turns to face it on its own and the driver keeps control of where it drives. Moving the turn stick takes back control.
```java
input.headingOverride(() -> controller.getRightBumperButton()
        ? Optional.of(AimSolver.fieldAngle(drive.getPose(), goal.get2d()))   // or ShootOnTheMove
        : Optional.empty());
```

> In simulation, the Driver Station defaults to the **red** alliance, so field-relative driving is flipped until you pick a blue station in the sim GUI.

`PoseController` drives a swerve robot to a pose in a straight line, e.g. auto-aligning to the nearest scoring spot:
```java
PoseController align = new PoseController();
controller.a().whileTrue(align.driveTo(
    drive::getPose,
    () -> GeomUtil.nearest(drive.getPose(), scoringPoses),
    drive::driveRobotRelative,
    drive));
```

`SkidDetector` flags a swerve wheel that disagrees with the others (slipping or being pushed), e.g. to trust vision over odometry for a moment. `WheelRadiusCharacterization` spins the robot in place to measure your real wheel radius; see its Javadoc.

## auto
`AutoStartChecker` compares the robot's pose (from vision) with the selected auto's start pose while disabled, and shows an alert like *"Robot is 0.42 m / 12° from the auto's start pose"*. Light up the LEDs when it's in place so the drive team can see it from behind the glass:
```java
AutoStartChecker startCheck = new AutoStartChecker(
    drive::getPose,
    () -> Optional.ofNullable(startPoses.get(autoChooser.getSelected())));   // blue-side poses; flipped on red
startCheck.ready().and(DriverStation::isDisabled).whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 5));
```

## input
```java
ButtonPatterns.doubleTap(controller.a(), 0.3).onTrue(scoreHigh());
ButtonPatterns.tap(controller.b(), 0.4).onTrue(toggleIntake());
ButtonPatterns.hold(controller.b(), 0.4).onTrue(ejectEverything());
ButtonPatterns.chord(controller.leftBumper(), controller.rightBumper()).onTrue(climb());

hasPiece.onTrue(Rumble.rumble(controller, 0.8, 0.3));
MatchTimer.teleopTimeLeftBelow(20).onTrue(Rumble.pulses(controller, 1, 3));
```
`InputCurves` has the deadband and curve functions `DriveInput` uses, for your own axes.

## commands
```java
enum State { IDLE, INTAKING, HOLDING }
StateMachine<State> intake = new StateMachine<>(State.IDLE)
    .onEnter(State.INTAKING, () -> roller.setVoltage(8))
    .onExit(State.INTAKING, () -> roller.setVoltage(0))
    .transition(State.INTAKING, State.HOLDING, hasPiece);
// call intake.update() in periodic(), and intake.setState(State.INTAKING) from a button

Trigger slowMode = TriggerUtil.toggle(controller.leftStick(), false);
Command grab = CommandUtil.waitUntilOrTimeout(intake::hasPiece, 2.0, () -> DriverStation.reportWarning("No piece", false));
Command logged = CommandUtil.logged("ScoreHigh", scoreHigh());   // prints start/end/interrupt with timing
```

## mechanisms
```java
// True once the intake rollers stall on a game piece.
Trigger hasPiece = StallDetector.trigger(
    () -> intakeMotor.getStatorCurrent().getValueAsDouble(), 30,
    () -> intakeMotor.getVelocity().getValueAsDouble(), 2,
    0.1);

// Drive the elevator gently down until it hits the bottom, then zero the encoder.
Command home = Homing.toHardStop(elevator::setVoltage, () -> motor.getStatorCurrent().getValueAsDouble(),
    -1.5, 20, () -> motor.setPosition(0), 3.0, elevator);
```

**SysId** (measures kS, kV, kA and kG so you don't have to guess feedforward gains):
```java
SysIdRoutine shooterId = SysIdHelper.rotary("Shooter",
    shooter::setVoltage,
    () -> motor.getPosition().getValueAsDouble(),     // rotations
    () -> motor.getVelocity().getValueAsDouble(),     // rotations per second
    shooter);
SysIdHelper.bind(shooterId, controller.a(), controller.b(), controller.x(), controller.y());
```
Hold each button until the mechanism nears its limit, then open the log in the SysId tool. Use `SysIdHelper.linear(...)` for elevators and drivetrains (meters), and `SysIdHelper.runAll(routine)` to run all four tests in a row on a flywheel.

## leds
```java
LEDController leds = new LEDController(0, 60);   // PWM port, LED count
leds.setDefault(LEDPattern.solid(Color.kBlue));
hasPiece.whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 1));
readyToShoot.whileTrue(leds.show(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1)), 2));
```
The highest priority request shows; when it ends, the next one down comes back.

## vision
`VisionFilter` throws out bad AprilTag estimates (off the field, floating, ambiguous, too far) and tells your pose estimator how much to trust the rest: close, multi-tag views count most.
```java
VisionFilter filter = new VisionFilter();
filter.process(estimate.estimatedPose, estimate.timestampSeconds, tagCount, avgTagDistance, ambiguity)
    .ifPresent(m -> poseEstimator.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs()));
```

`ObjectDetection.toField(pose, robotToCamera, yaw, pitch, pieceHeight)` turns a detected game piece's angles in the image into its position on the field. `GamePieceTracker` remembers those positions, so the robot can keep driving to a piece when the camera loses it for a moment:
```java
detection.ifPresent(p -> pieces.add(p, timestamp));
pieces.removeOlderThan(Timer.getFPGATimestamp());
Optional<Translation2d> target = pieces.closest(drive.getPose());
```

## diagnostics
`HealthMonitor` shows device problems as dashboard alerts (Elastic, Shuffleboard, AdvantageScope). Brief disconnects stay listed ("FL Drive was disconnected 3 times") so you can find loose wires after the match:
```java
HealthMonitor.connected("FL Drive", flDrive::isConnected);           // CTRE
HealthMonitor.connected("Intake", () -> !intake.getFaults().can);    // REV
HealthMonitor.temperature("Shooter", () -> shooter.getDeviceTemp().getValueAsDouble(), 70);
HealthMonitor.lowBattery(12.3);          // resting voltage, checked while disabled
HealthMonitor.canUtilization(0.9);
HealthMonitor.check("Elevator not homed", elevator::isHomed, AlertType.kWarning);
```

## robot
```java
MatchTimer.endgame().onTrue(Rumble.pulses(controller, 1, 2));

Constants constants = new RobotIdentity<>(COMP_CONSTANTS)
    .add("0316B37A", PRACTICE_CONSTANTS)   // practice robot's roboRIO serial number
    .get();
```

## season2026
Code that only applies to this year's game. It'll be removed when the next season's version comes out.

`HubShift` knows whether your hub is active during REBUILT's alliance shifts, from the match time and the FMS game data:
```java
HubShift.hubActive().whileTrue(leds.show(LEDPattern.solid(Color.kGreen), 3));
boolean arrivesActive = HubShift.isHubActiveIn(shot.timeOfFlight());   // fuel in the air counts when it lands
double secondsLeft = HubShift.timeUntilChange();
```
Match time is approximate, so use this for driver feedback rather than to block shooting.
