package io.github.jamesehart.lightutil.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import io.github.jamesehart.lightutil.field.AllianceFlip;
import io.github.jamesehart.lightutil.input.InputCurves;

/**
 * Turns driver joystick values into drivetrain speeds, with deadband, response curve, acceleration
 * limit, slow mode and aim assist.
 *
 * <p>Field-relative driving is alliance-aware: pushing the stick forward always drives away from
 * your own driver station, on either alliance.
 *
 * <pre>{@code
 * DriveInput input = new DriveInput(
 *         () -> -controller.getLeftY(),    // forward
 *         () -> -controller.getLeftX(),    // left
 *         () -> -controller.getRightX())   // counter-clockwise
 *     .maxSpeed(4.5)
 *     .maxTurnRate(2 * Math.PI)
 *     .slowMode(controller::getLeftBumperButton, 0.4);
 *
 * drive.driveRobotRelative(input.fieldRelative(drive.getPose().getRotation()));
 * }</pre>
 *
 * <p><b>Aim assist:</b> {@link #headingOverride} lets the robot face something (a goal, a game
 * piece) automatically while the driver keeps full control of where it drives:
 *
 * <pre>{@code
 * input.headingOverride(() -> controller.getHID().getRightBumperButton()
 *     ? Optional.of(AimSolver.fieldAngle(drive.getPose(), goal.get2d()))
 *     : Optional.empty());
 * }</pre>
 */
public final class DriveInput {
    private final DoubleSupplier forward;
    private final DoubleSupplier left;
    private final DoubleSupplier turn;

    private double maxSpeed = 4.5;
    private double maxTurnRate = 2 * Math.PI;
    private double deadband = 0.1;
    private double exponent = 2;
    private BooleanSupplier slowMode = () -> false;
    private double slowScale = 0.4;
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter turnLimiter;
    private DoubleSupplier speedScale = () -> 1;
    private Supplier<Optional<Rotation2d>> headingOverride = Optional::empty;
    private boolean driverCanOverride = true;
    private final PIDController headingController = new PIDController(5, 0, 0);
    private boolean overrideActive = false;

    /**
     * Creates a drive input from three stick axes, each in [-1, 1].
     *
     * @param forward positive drives forward (usually {@code -controller.getLeftY()})
     * @param left positive drives left (usually {@code -controller.getLeftX()})
     * @param turn positive turns counter-clockwise (usually {@code -controller.getRightX()})
     */
    public DriveInput(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier turn) {
        this.forward = forward;
        this.left = left;
        this.turn = turn;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** Speed at full stick, m/s. Default 4.5. */
    public DriveInput maxSpeed(double metersPerSecond) {
        this.maxSpeed = metersPerSecond;
        return this;
    }

    /** Turn rate at full stick, rad/s. Default 2π. */
    public DriveInput maxTurnRate(double radiansPerSecond) {
        this.maxTurnRate = radiansPerSecond;
        return this;
    }

    /** Stick deadband. Default 0.1. */
    public DriveInput deadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    /** Response curve exponent: 1 is linear, 2 (default) is squared, 3 is cubic. */
    public DriveInput exponent(double exponent) {
        this.exponent = exponent;
        return this;
    }

    /**
     * Limits how fast the commanded speeds can change, to stop the robot tipping or wheels slipping.
     * Off by default.
     *
     * @param metersPerSecondSquared translation acceleration limit
     * @param radiansPerSecondSquared rotation acceleration limit
     * @return this, for chaining
     */
    public DriveInput accelerationLimit(double metersPerSecondSquared, double radiansPerSecondSquared) {
        this.xLimiter = new SlewRateLimiter(metersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(metersPerSecondSquared);
        this.turnLimiter = new SlewRateLimiter(radiansPerSecondSquared);
        return this;
    }

    /**
     * Scales everything down while a button is held.
     *
     * @param active true while slow mode should be on
     * @param scale speed multiplier in slow mode, e.g. 0.4
     * @return this, for chaining
     */
    public DriveInput slowMode(BooleanSupplier active, double scale) {
        this.slowMode = active;
        this.slowScale = scale;
        return this;
    }

    /**
     * Scales speed continuously, e.g. from a trigger: {@code () -> 1 - 0.7 *
     * controller.getLeftTriggerAxis()} slows down the harder the trigger is pulled. Multiplies
     * with slow mode.
     *
     * @param scale 0 to 1 (clamped)
     * @return this, for chaining
     */
    public DriveInput speedScale(DoubleSupplier scale) {
        this.speedScale = scale;
        return this;
    }

    /**
     * Aim assist: while {@code heading} returns a value, the robot turns to face that field heading
     * on its own, while the driver still controls translation. Only applies to {@link
     * #fieldRelative}. Use {@code AimSolver.fieldAngle} or {@code ShootOnTheMove} for the heading.
     *
     * @param heading the field heading to face, or empty for normal driving
     * @return this, for chaining
     */
    public DriveInput headingOverride(Supplier<Optional<Rotation2d>> heading) {
        this.headingOverride = heading;
        return this;
    }

    /**
     * Tunes the aim assist heading controller. Default kP 5, kD 0.
     *
     * @param kP rad/s per radian of heading error
     * @param kD rad/s per rad/s of heading error rate
     * @param driverCanOverride if true (the default), moving the turn stick takes back control
     * @return this, for chaining
     */
    public DriveInput headingController(double kP, double kD, boolean driverCanOverride) {
        headingController.setPID(kP, 0, kD);
        this.driverCanOverride = driverCanOverride;
        return this;
    }

    /** True if aim assist was controlling the heading at the last {@link #fieldRelative} call. */
    public boolean isHeadingOverridden() {
        return overrideActive;
    }

    /**
     * Robot-relative speeds for field-relative driving: the stick direction is a direction on the
     * field, from the driver's point of view.
     *
     * @param robotHeading the robot's field heading (from odometry or the gyro)
     * @return robot-relative speeds to send to the drivetrain
     */
    public ChassisSpeeds fieldRelative(Rotation2d robotHeading) {
        ChassisSpeeds speeds = shaped();
        // The red driver stands at the other end of the field, facing -x.
        Rotation2d driverFacing = AllianceFlip.isRed() ? Rotation2d.kPi : Rotation2d.kZero;
        Translation2d field = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                .rotateBy(driverFacing);
        ChassisSpeeds limited = limit(new ChassisSpeeds(field.getX(), field.getY(), speeds.omegaRadiansPerSecond));

        Optional<Rotation2d> target = headingOverride.get();
        boolean driverTurning = Math.abs(turn.getAsDouble()) > deadband;
        boolean override = target.isPresent() && !(driverCanOverride && driverTurning);
        if (override) {
            if (!overrideActive) headingController.reset();
            double omega = headingController.calculate(robotHeading.getRadians(), target.get().getRadians());
            limited = new ChassisSpeeds(limited.vxMetersPerSecond, limited.vyMetersPerSecond,
                    MathUtil.clamp(omega, -maxTurnRate, maxTurnRate));
        }
        overrideActive = override;
        return ChassisSpeeds.fromFieldRelativeSpeeds(limited, robotHeading);
    }

    /** Robot-relative speeds for robot-relative driving: the stick direction is relative to the robot's front. */
    public ChassisSpeeds robotRelative() {
        return limit(shaped());
    }

    private ChassisSpeeds shaped() {
        Translation2d stick = InputCurves.stick(forward.getAsDouble(), left.getAsDouble(), deadband, exponent);
        double rot = InputCurves.power(InputCurves.deadband(turn.getAsDouble(), deadband), exponent);
        double scale = MathUtil.clamp(speedScale.getAsDouble(), 0, 1) * (slowMode.getAsBoolean() ? slowScale : 1);
        return new ChassisSpeeds(
                stick.getX() * maxSpeed * scale,
                stick.getY() * maxSpeed * scale,
                rot * maxTurnRate * scale);
    }

    private ChassisSpeeds limit(ChassisSpeeds speeds) {
        if (xLimiter == null) return speeds;
        return new ChassisSpeeds(
                xLimiter.calculate(speeds.vxMetersPerSecond),
                yLimiter.calculate(speeds.vyMetersPerSecond),
                turnLimiter.calculate(speeds.omegaRadiansPerSecond));
    }
}
