package io.github.jamesehart.lightsim.physics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import io.github.jamesehart.lightsim.SimGyro;
import io.github.jamesehart.lightsim.SimLoad;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * A drivetrain as a rigid body in the {@link PhysicsWorld}. Each wheel pushes the body with the
 * force its motors produce, limited by carpet traction, and sideways sliding is resisted by friction.
 * Collisions with walls and obstacles are handled by dyn4j.
 */
public final class DrivetrainSim implements SimLoad {
    private static final int SUBSTEPS = 5;
    private static final double GRAVITY = 9.81;

    private static final class Wheel {
        final Translation2d location;
        final MotorGroup drive;
        final RotaryMechanism steer;
        double wheelAngleRad;
        double wheelSpeedMps;

        Wheel(Translation2d location, MotorGroup drive, RotaryMechanism steer) {
            this.location = location;
            this.drive = drive;
            this.steer = steer;
        }

        double moduleAngleRad() {
            return steer == null ? 0 : steer.getPosition();
        }
    }

    private final PhysicsWorld world;
    private final Body body = new Body();
    private final Wheel[] wheels;
    private final boolean swerve;
    private final double wheelRadius;
    private final double massPerWheel;
    private final double maxTractionPerWheel;
    private final SimGyro gyro;

    private double lastHeadingRad;
    private double unwrappedYawRad;
    private double supplyCurrent;

    private DrivetrainSim(
            PhysicsWorld world,
            Wheel[] wheels,
            boolean swerve,
            double wheelRadius,
            double massKg,
            double lengthM,
            double widthM,
            double cof,
            SimGyro gyro) {
        this.world = world;
        this.wheels = wheels;
        this.swerve = swerve;
        this.wheelRadius = wheelRadius;
        this.massPerWheel = massKg / wheels.length;
        this.maxTractionPerWheel = cof * massPerWheel * GRAVITY;
        this.gyro = gyro;

        BodyFixture fixture = body.addFixture(Geometry.createRectangle(lengthM, widthM));
        fixture.setDensity(massKg / (lengthM * widthM));
        fixture.setFriction(0.5);
        fixture.setRestitution(0.1);
        body.setMass(MassType.NORMAL);
        body.setAtRestDetectionEnabled(false);
        body.setBullet(true);
        world.getWorld().addBody(body);
        setPose(new Pose2d(world.getFieldLength() / 2, world.getFieldWidth() / 2, Rotation2d.kZero));
    }

    /** Creates a swerve drivetrain in {@code world}, starting at the center of the field. */
    public static DrivetrainSim swerve(PhysicsWorld world, SwerveSimConfig config) {
        if (config.modules.isEmpty()) throw new IllegalArgumentException("Swerve needs at least one module");
        Wheel[] wheels = config.modules.stream()
                .map(m -> new Wheel(
                        m.location(),
                        new MotorGroup(config.driveGearing, m.drive()),
                        RotaryMechanism.steering(new MotorGroup(config.steerGearing, m.steer()), config.steerMoiKgM2)))
                .toArray(Wheel[]::new);
        return new DrivetrainSim(world, wheels, true, config.wheelRadiusM, config.robotMassKg,
                config.bumperLengthM, config.bumperWidthM, config.wheelCof, config.gyro);
    }

    /** Creates a tank drivetrain in {@code world}, starting at the center of the field. */
    public static DrivetrainSim tank(PhysicsWorld world, TankSimConfig config) {
        if (config.left.length == 0 || config.right.length == 0) {
            throw new IllegalArgumentException("Tank needs at least one left and one right motor");
        }
        Wheel[] wheels = {
            new Wheel(new Translation2d(0, config.trackWidthM / 2), new MotorGroup(config.gearing, config.left), null),
            new Wheel(new Translation2d(0, -config.trackWidthM / 2), new MotorGroup(config.gearing, config.right), null)
        };
        return new DrivetrainSim(world, wheels, false, config.wheelRadiusM, config.robotMassKg,
                config.bumperLengthM, config.bumperWidthM, config.wheelCof, config.gyro);
    }

    @Override
    public void step(double dt, double supplyVolts) {
        supplyCurrent = 0;
        for (Wheel wheel : wheels) {
            if (wheel.steer != null) {
                wheel.steer.step(dt, supplyVolts);
                supplyCurrent += wheel.steer.getSupplyCurrentAmps();
            }
            wheel.drive.readVolts(supplyVolts);
        }

        double h = dt / SUBSTEPS;
        for (int i = 0; i < SUBSTEPS; i++) {
            applyWheelForces(h);
            world.getWorld().step(1, h);
        }

        for (Wheel wheel : wheels) {
            double wheelOmega = wheel.wheelSpeedMps / wheelRadius;
            wheel.drive.writeState(wheel.wheelAngleRad, wheelOmega, supplyVolts, dt);
            supplyCurrent += wheel.drive.supplyCurrent(wheelOmega, supplyVolts);
        }

        double heading = body.getTransform().getRotationAngle();
        unwrappedYawRad += Math.IEEEremainder(heading - lastHeadingRad, 2 * Math.PI);
        lastHeadingRad = heading;
        if (gyro != null) {
            gyro.setSimulatedYaw(Math.toDegrees(unwrappedYawRad), Math.toDegrees(body.getAngularVelocity()));
        }
    }

    private void applyWheelForces(double h) {
        double heading = body.getTransform().getRotationAngle();
        Vector2 center = body.getWorldCenter();
        Vector2 v = body.getLinearVelocity();
        double omega = body.getAngularVelocity();

        for (Wheel wheel : wheels) {
            // Wheel contact point, and its velocity over the carpet.
            Translation2d r = wheel.location.rotateBy(new Rotation2d(heading));
            double vx = v.x - omega * r.getY();
            double vy = v.y + omega * r.getX();

            // Rolling direction and sideways direction of the wheel.
            double wheelDir = heading + wheel.moduleAngleRad();
            double ux = Math.cos(wheelDir);
            double uy = Math.sin(wheelDir);
            double groundSpeed = vx * ux + vy * uy;
            double slipSpeed = -vx * uy + vy * ux;

            // Motor force along the wheel: F = A - B·(wheel surface speed).
            double a = wheel.drive.driveTorque() / wheelRadius;
            double b = wheel.drive.damping() / (wheelRadius * wheelRadius);
            double rolling = a - b * groundSpeed;
            // Friction stops sideways sliding, as much as traction allows.
            double sideways = -massPerWheel * slipSpeed / h;

            double total = Math.hypot(rolling, sideways);
            if (total > maxTractionPerWheel) {
                double scale = maxTractionPerWheel / total;
                rolling *= scale;
                sideways *= scale;
            }

            // When traction runs out the wheel spins faster than the ground moves.
            wheel.wheelSpeedMps = b > 1e-9 ? (a - rolling) / b : groundSpeed;
            wheel.wheelAngleRad += wheel.wheelSpeedMps / wheelRadius * h;

            double fx = rolling * ux - sideways * uy;
            double fy = rolling * uy + sideways * ux;
            body.applyForce(new Vector2(fx, fy), new Vector2(center.x + r.getX(), center.y + r.getY()));
        }
    }

    @Override
    public double getSupplyCurrentAmps() {
        return supplyCurrent;
    }

    /** The true robot pose on the field. */
    public Pose2d getPose() {
        Vector2 t = body.getTransform().getTranslation();
        return new Pose2d(t.x, t.y, new Rotation2d(body.getTransform().getRotationAngle()));
    }

    /** Teleports the robot, stopping it. */
    public void setPose(Pose2d pose) {
        body.getTransform().identity();
        body.rotate(pose.getRotation().getRadians());
        body.translate(pose.getX(), pose.getY());
        body.setLinearVelocity(0, 0);
        body.setAngularVelocity(0);
        lastHeadingRad = pose.getRotation().getRadians();
        unwrappedYawRad = lastHeadingRad;
    }

    /** Continuous yaw in radians, counter-clockwise positive. */
    public double getYawRad() {
        return unwrappedYawRad;
    }

    /** Yaw rate in rad/s, counter-clockwise positive. */
    public double getYawRateRadPerSec() {
        return body.getAngularVelocity();
    }

    /** Every drive and steering motor in this drivetrain. */
    public SimMotor[] getMotors() {
        List<SimMotor> all = new ArrayList<>();
        for (Wheel wheel : wheels) {
            all.addAll(Arrays.asList(wheel.drive.getMotors()));
            if (wheel.steer != null) all.addAll(Arrays.asList(wheel.steer.getMotors().getMotors()));
        }
        return all.toArray(SimMotor[]::new);
    }

    /** True if this is a swerve drivetrain. */
    public boolean isSwerve() {
        return swerve;
    }

    /** Each wheel's speed and module angle, for AdvantageScope's swerve tab. */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[wheels.length];
        for (int i = 0; i < wheels.length; i++) {
            states[i] = new SwerveModuleState(wheels[i].wheelSpeedMps, new Rotation2d(wheels[i].moduleAngleRad()));
        }
        return states;
    }
}
