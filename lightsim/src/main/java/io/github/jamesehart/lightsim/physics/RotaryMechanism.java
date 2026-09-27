package io.github.jamesehart.lightsim.physics;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import io.github.jamesehart.lightsim.SimLoad;

/**
 * A single-axis mechanism: flywheel, roller, arm, elevator, swerve steering, or a motor with nothing
 * attached. Created through {@link #flywheel}, {@link #elevator}, {@link #arm} or {@link #freeLoad}.
 *
 * <p>Positions and speeds are in "output units": radians for rotating mechanisms, meters for an
 * elevator.
 */
public final class RotaryMechanism implements SimLoad {
    private static final double GRAVITY = 9.81;

    private final MotorGroup motors;
    private final double moi;
    private final double outputScale;
    private final DoubleUnaryOperator gravityTorque;
    private final double minRad;
    private final double maxRad;
    private final boolean linear;

    private int componentIndex = -1;
    private Translation3d componentOrigin = Translation3d.kZero;
    private Translation3d componentAxis;

    private double angle;
    private double omega;
    private double supplyCurrent;

    private RotaryMechanism(
            MotorGroup motors,
            double moi,
            double outputScale,
            DoubleUnaryOperator gravityTorque,
            double minRad,
            double maxRad,
            double startRad,
            boolean linear) {
        this.motors = motors;
        this.moi = moi;
        this.outputScale = outputScale;
        this.gravityTorque = gravityTorque;
        this.minRad = minRad;
        this.maxRad = maxRad;
        this.angle = startRad;
        this.linear = linear;
        this.componentAxis = linear ? new Translation3d(0, 0, 1) : new Translation3d(0, -1, 0);
    }

    /**
     * A spinning mechanism with no gravity or limits (flywheel, roller, intake).
     *
     * @param motors the motors and gearing
     * @param moiKgM2 moment of inertia of the spinning part, in kg·m²
     */
    public static RotaryMechanism flywheel(MotorGroup motors, double moiKgM2) {
        return new RotaryMechanism(
                motors, moiKgM2, 1, a -> 0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0, false);
    }

    /** A light load so a motor with nothing attached still spins and reports sensor data. */
    public static RotaryMechanism freeLoad(MotorGroup motors) {
        return flywheel(motors, 0.001);
    }

    /**
     * A vertical elevator driven by a drum or sprocket. Starts at {@code minHeightM}.
     *
     * @param motors the motors and gearing
     * @param carriageMassKg mass being lifted
     * @param drumRadiusM radius of the drum/sprocket
     * @param minHeightM bottom hard stop
     * @param maxHeightM top hard stop
     */
    public static RotaryMechanism elevator(
            MotorGroup motors, double carriageMassKg, double drumRadiusM, double minHeightM, double maxHeightM) {
        double gravity = -carriageMassKg * GRAVITY * drumRadiusM;
        return new RotaryMechanism(
                motors,
                carriageMassKg * drumRadiusM * drumRadiusM,
                drumRadiusM,
                a -> gravity,
                minHeightM / drumRadiusM,
                maxHeightM / drumRadiusM,
                minHeightM / drumRadiusM,
                true);
    }

    /**
     * A single-jointed arm, treated as a uniform rod. 0 rad is horizontal, positive is up. Starts at
     * {@code minRad}.
     *
     * @param motors the motors and gearing
     * @param moiKgM2 moment of inertia about the pivot, in kg·m²
     * @param armLengthM length from pivot to tip
     * @param minRad lower hard stop
     * @param maxRad upper hard stop
     */
    public static RotaryMechanism arm(
            MotorGroup motors, double moiKgM2, double armLengthM, double minRad, double maxRad) {
        // Uniform rod about one end: J = m·L²/3, center of mass at L/2.
        double mass = 3 * moiKgM2 / (armLengthM * armLengthM);
        double maxGravity = mass * GRAVITY * armLengthM / 2;
        return new RotaryMechanism(motors, moiKgM2, 1, a -> -maxGravity * Math.cos(a), minRad, maxRad, minRad, false);
    }

    /** Swerve module steering, no limits. */
    static RotaryMechanism steering(MotorGroup motors, double moiKgM2) {
        return flywheel(motors, moiKgM2);
    }

    @Override
    public void step(double dt, double supplyVolts) {
        motors.readVolts(supplyVolts);
        double startOmega = omega;

        // J·ω' = A + τg - B·ω, with τg held constant over the step, integrated exactly.
        double a = motors.driveTorque() + gravityTorque.applyAsDouble(angle);
        double b = motors.damping();
        if (b < 1e-9) {
            omega += a / moi * dt;
            angle += (startOmega + omega) / 2 * dt;
        } else {
            double k = b / moi;
            double steady = a / b;
            double decay = Math.exp(-k * dt);
            omega = steady + (startOmega - steady) * decay;
            angle += steady * dt + (startOmega - steady) * (1 - decay) / k;
        }

        if (angle <= minRad) {
            angle = minRad;
            omega = Math.max(omega, 0);
        } else if (angle >= maxRad) {
            angle = maxRad;
            omega = Math.min(omega, 0);
        }

        supplyCurrent = motors.supplyCurrent(omega, supplyVolts);
        motors.writeState(angle, omega, supplyVolts, dt);
    }

    @Override
    public double getSupplyCurrentAmps() {
        return supplyCurrent;
    }

    /** Position in output units (rad, or meters for an elevator). */
    public double getPosition() {
        return angle * outputScale;
    }

    /** Velocity in output units per second (rad/s, or m/s for an elevator). */
    public double getVelocity() {
        return omega * outputScale;
    }

    /**
     * Shows this mechanism as a moving part of an AdvantageScope robot model. Its pose is published
     * at {@code index} in {@code /LightSim/Components}, matching the order of "components" in the
     * model's config.json.
     *
     * <p>Elevators move up (+z) from {@code origin} by their height above the bottom stop. Everything
     * else rotates about {@code origin}: arms around -y, so positive angles lift the front.
     *
     * @param index the component's index in the robot model
     * @param origin robot-relative position of the component at zero (pivot for arms), meters
     * @return this, for chaining
     */
    public RotaryMechanism component(int index, Translation3d origin) {
        componentIndex = index;
        componentOrigin = origin;
        return this;
    }

    /**
     * Like {@link #component(int, Translation3d)}, but with a custom axis: the direction an elevator
     * moves, or the axis a rotating part turns around (e.g. {@code (0, 0, 1)} for a turret).
     *
     * @param index the component's index in the robot model
     * @param origin robot-relative position of the component at zero, meters
     * @param axis movement/rotation axis, robot-relative
     * @return this, for chaining
     */
    public RotaryMechanism component(int index, Translation3d origin, Translation3d axis) {
        component(index, origin);
        componentAxis = axis.div(axis.getNorm());
        return this;
    }

    /** The AdvantageScope component index, or -1 if this isn't shown as a component. */
    public int getComponentIndex() {
        return componentIndex;
    }

    /** Robot-relative pose of this mechanism's AdvantageScope component. */
    public Pose3d getComponentPose() {
        if (linear) {
            double travel = (angle - minRad) * outputScale;
            return new Pose3d(componentOrigin.plus(componentAxis.times(travel)), Rotation3d.kZero);
        }
        double half = angle / 2;
        double sin = Math.sin(half);
        Rotation3d rotation = new Rotation3d(new Quaternion(
                Math.cos(half), componentAxis.getX() * sin, componentAxis.getY() * sin, componentAxis.getZ() * sin));
        return new Pose3d(componentOrigin, rotation);
    }

    MotorGroup getMotors() {
        return motors;
    }
}
