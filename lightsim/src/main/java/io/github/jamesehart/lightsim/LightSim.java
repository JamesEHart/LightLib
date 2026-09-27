package io.github.jamesehart.lightsim;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import io.github.jamesehart.lightsim.physics.DrivetrainSim;
import io.github.jamesehart.lightsim.physics.MotorGroup;
import io.github.jamesehart.lightsim.physics.PhysicsWorld;
import io.github.jamesehart.lightsim.physics.RotaryMechanism;
import io.github.jamesehart.lightsim.physics.SwerveSimConfig;
import io.github.jamesehart.lightsim.physics.TankSimConfig;

/**
 * Simple physics simulation for FRC robots.
 *
 * <p>Call {@link #enable(TimedRobot)} at the top of your Robot constructor, and use the {@code Sim*}
 * motor classes (e.g. {@code SimTalonFX}) instead of the vendor ones. In simulation every motor then
 * gets physics; on a real robot LightSim does nothing.
 *
 * <p>Everything LightSim simulates is published under {@code /LightSim/} for AdvantageScope.
 */
public final class LightSim {
    private static final double PERIOD = 0.02;
    private static final double FALLBACK_FIELD_LENGTH = 16.54;
    private static final double FALLBACK_FIELD_WIDTH = 8.07;

    private static final List<SimMotor> motors = new ArrayList<>();
    private static final Set<SimMotor> claimed = Collections.newSetFromMap(new IdentityHashMap<>());
    private static final Map<SimMotor, RotaryMechanism> freeLoads = new IdentityHashMap<>();
    private static final List<SimLoad> loads = new ArrayList<>();
    private static final List<RotaryMechanism> mechanisms = new ArrayList<>();

    private static PhysicsWorld world;
    private static DrivetrainSim drivetrain;
    private static boolean enabled = false;
    private static double supplyCurrent = 0;

    private static StructPublisher<Pose2d> posePub;
    private static StructPublisher<Pose3d> pose3dPub;
    private static StructArrayPublisher<SwerveModuleState> swervePub;
    private static StructArrayPublisher<Pose3d> componentsPub;
    private static DoublePublisher batteryPub;

    private LightSim() {}

    /**
     * Turns on the simulation. Call once, at the top of your Robot constructor. Does nothing on a real
     * robot.
     *
     * @param robot your robot, usually {@code this}
     */
    public static void enable(TimedRobot robot) {
        if (enabled || !RobotBase.isSimulation()) return;
        enabled = true;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("LightSim");
        posePub = table.getStructTopic("RobotPose", Pose2d.struct).publish();
        pose3dPub = table.getStructTopic("RobotPose3d", Pose3d.struct).publish();
        swervePub = table.getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
        componentsPub = table.getStructArrayTopic("Components", Pose3d.struct).publish();
        batteryPub = table.getDoubleTopic("BatteryVoltage").publish();

        robot.addPeriodic(LightSim::update, PERIOD);
    }

    /**
     * Adds a motor to the simulation. The {@code Sim*} classes call this themselves; you only need it
     * for your own {@link SimMotor}.
     *
     * @param motor the motor
     */
    public static void register(SimMotor motor) {
        motors.add(motor);
    }

    // ---------------------------------------------------------------- mechanisms

    /**
     * Simulates a flywheel, roller or intake: something that just spins.
     *
     * @param motor the motor (a {@code Sim*} motor controller)
     * @param gearing reduction from motor to wheel, e.g. 2.0 for 2:1
     * @param moiKgM2 moment of inertia of the spinning part, in kg·m²
     * @return the mechanism, e.g. to read its speed or show it in AdvantageScope
     */
    public static RotaryMechanism flywheel(SimMotor motor, double gearing, double moiKgM2) {
        return flywheel(new SimMotor[] {motor}, gearing, moiKgM2);
    }

    /** Same as {@link #flywheel(SimMotor, double, double)}, for several motors on one shaft. */
    public static RotaryMechanism flywheel(SimMotor[] motors, double gearing, double moiKgM2) {
        return addMechanism(RotaryMechanism.flywheel(claim(gearing, motors), moiKgM2));
    }

    /**
     * Simulates a vertical elevator. It starts at the bottom.
     *
     * @param motor the motor (a {@code Sim*} motor controller)
     * @param gearing reduction from motor to drum
     * @param carriageMassKg mass being lifted
     * @param drumRadiusM radius of the drum or sprocket
     * @param minHeightM bottom hard stop
     * @param maxHeightM top hard stop
     * @return the mechanism, e.g. to read its height or show it in AdvantageScope
     */
    public static RotaryMechanism elevator(SimMotor motor, double gearing, double carriageMassKg,
            double drumRadiusM, double minHeightM, double maxHeightM) {
        return elevator(new SimMotor[] {motor}, gearing, carriageMassKg, drumRadiusM, minHeightM, maxHeightM);
    }

    /** Same as {@link #elevator(SimMotor, double, double, double, double, double)}, for several motors. */
    public static RotaryMechanism elevator(SimMotor[] motors, double gearing, double carriageMassKg,
            double drumRadiusM, double minHeightM, double maxHeightM) {
        return addMechanism(RotaryMechanism.elevator(
                claim(gearing, motors), carriageMassKg, drumRadiusM, minHeightM, maxHeightM));
    }

    /**
     * Simulates a single-jointed arm with gravity. 0 rad is horizontal, positive is up. It starts at
     * {@code minRad}.
     *
     * @param motor the motor (a {@code Sim*} motor controller)
     * @param gearing reduction from motor to arm
     * @param moiKgM2 moment of inertia about the pivot, in kg·m²
     * @param armLengthM length from pivot to tip
     * @param minRad lower hard stop
     * @param maxRad upper hard stop
     * @return the mechanism, e.g. to read its angle or show it in AdvantageScope
     */
    public static RotaryMechanism arm(SimMotor motor, double gearing, double moiKgM2, double armLengthM,
            double minRad, double maxRad) {
        return arm(new SimMotor[] {motor}, gearing, moiKgM2, armLengthM, minRad, maxRad);
    }

    /** Same as {@link #arm(SimMotor, double, double, double, double, double)}, for several motors. */
    public static RotaryMechanism arm(SimMotor[] motors, double gearing, double moiKgM2, double armLengthM,
            double minRad, double maxRad) {
        return addMechanism(RotaryMechanism.arm(claim(gearing, motors), moiKgM2, armLengthM, minRad, maxRad));
    }

    // ---------------------------------------------------------------- drivetrains and field

    /**
     * Simulates a swerve drivetrain on the field. It starts in the middle of the field; see {@link
     * #setRobotPose(Pose2d)}.
     *
     * @param config the modules and robot dimensions
     * @return the drivetrain
     */
    public static DrivetrainSim swerve(SwerveSimConfig config) {
        return setDrivetrain(DrivetrainSim.swerve(world(), config));
    }

    /**
     * Simulates a tank (differential) drivetrain on the field. It starts in the middle of the field;
     * see {@link #setRobotPose(Pose2d)}.
     *
     * @param config the motors and robot dimensions
     * @return the drivetrain
     */
    public static DrivetrainSim tank(TankSimConfig config) {
        return setDrivetrain(DrivetrainSim.tank(world(), config));
    }

    /** Teleports the simulated robot, e.g. to the auto starting position. */
    public static void setRobotPose(Pose2d pose) {
        if (drivetrain != null) drivetrain.setPose(pose);
    }

    /** The true simulated robot pose, or the origin if there's no drivetrain (or on a real robot). */
    public static Pose2d getRobotPose() {
        return drivetrain == null ? Pose2d.kZero : drivetrain.getPose();
    }

    /**
     * Adds an immovable box to the field that the robot collides with.
     *
     * @param center center and rotation of the box
     * @param lengthM size along the box's x
     * @param widthM size along the box's y
     */
    public static void addObstacle(Pose2d center, double lengthM, double widthM) {
        world().addObstacle(center.getX(), center.getY(), center.getRotation().getRadians(), lengthM, widthM);
    }

    // ---------------------------------------------------------------- internals

    private static PhysicsWorld world() {
        if (world == null) {
            double length = FALLBACK_FIELD_LENGTH;
            double width = FALLBACK_FIELD_WIDTH;
            try {
                // The same field AdvantageScope draws, so the walls line up with it.
                AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                length = field.getFieldLength();
                width = field.getFieldWidth();
            } catch (RuntimeException e) {
                System.out.println("LightSim: couldn't load the field size, using " + length + " x " + width);
            }
            world = new PhysicsWorld(length, width);
        }
        return world;
    }

    private static MotorGroup claim(double gearing, SimMotor... group) {
        for (SimMotor motor : group) {
            claimed.add(motor);
            RotaryMechanism free = freeLoads.remove(motor);
            if (free != null) loads.remove(free);
        }
        return new MotorGroup(gearing, group);
    }

    private static RotaryMechanism addMechanism(RotaryMechanism mechanism) {
        loads.add(mechanism);
        mechanisms.add(mechanism);
        return mechanism;
    }

    private static DrivetrainSim setDrivetrain(DrivetrainSim sim) {
        if (drivetrain != null) {
            throw new IllegalStateException("LightSim only supports one drivetrain");
        }
        claim(1, sim.getMotors());
        drivetrain = sim;
        loads.add(sim);
        return sim;
    }

    private static void update() {
        // Motors created after their mechanism was set up, or never given one, spin a light load
        // so their sensors still work.
        for (SimMotor motor : motors) {
            if (!claimed.contains(motor) && !freeLoads.containsKey(motor)) {
                RotaryMechanism free = RotaryMechanism.freeLoad(new MotorGroup(1, motor));
                freeLoads.put(motor, free);
                loads.add(free);
            }
        }

        double supplyVolts = Math.max(5, BatterySim.calculateDefaultBatteryLoadedVoltage(supplyCurrent));
        RoboRioSim.setVInVoltage(supplyVolts);

        supplyCurrent = 0;
        for (SimLoad load : loads) {
            load.step(PERIOD, supplyVolts);
            supplyCurrent += load.getSupplyCurrentAmps();
        }

        publish(supplyVolts);
    }

    private static void publish(double supplyVolts) {
        batteryPub.set(supplyVolts);
        if (drivetrain != null) {
            Pose2d pose = drivetrain.getPose();
            posePub.set(pose);
            pose3dPub.set(new Pose3d(pose));
            if (drivetrain.isSwerve()) swervePub.set(drivetrain.getModuleStates());
        }

        int count = 0;
        for (RotaryMechanism mechanism : mechanisms) {
            count = Math.max(count, mechanism.getComponentIndex() + 1);
        }
        if (count > 0) {
            Pose3d[] components = new Pose3d[count];
            Arrays.fill(components, Pose3d.kZero);
            for (RotaryMechanism mechanism : mechanisms) {
                if (mechanism.getComponentIndex() >= 0) {
                    components[mechanism.getComponentIndex()] = mechanism.getComponentPose();
                }
            }
            componentsPub.set(components);
        }
    }
}
