package io.github.jamesehart.lightutil.drive;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Measures your drive wheels' real radius (tread wears and compresses, so it's rarely the nominal
 * value). The robot spins in place; comparing how far the gyro turned with how far the wheels
 * rolled gives the effective radius. Run it on carpet for 10+ seconds, then cancel it and read the
 * result from the console or {@link #getRadiusMeters()}.
 *
 * <pre>{@code
 * controller.back().whileTrue(new WheelRadiusCharacterization(
 *     omega -> drive.driveRobotRelative(new ChassisSpeeds(0, 0, omega)),
 *     drive::getWheelPositionsRadians,   // each drive wheel's position, radians (after gearing)
 *     drive::getGyroRotation,
 *     Math.hypot(0.28, 0.28),            // center of robot to a module, meters
 *     drive));
 * }</pre>
 */
public final class WheelRadiusCharacterization extends Command {
    private final DoubleConsumer omegaOutput;
    private final Supplier<double[]> wheelPositionsRad;
    private final Supplier<Rotation2d> gyro;
    private final double driveBaseRadius;
    private final double omega;
    private final SlewRateLimiter ramp = new SlewRateLimiter(0.5);

    private double[] startPositions;
    private Rotation2d lastGyro;
    private double gyroAccumulated;
    private double radius = Double.NaN;

    /**
     * Creates the characterization, spinning at 1 rad/s.
     *
     * @param omegaOutput takes a robot turn rate in rad/s (counter-clockwise positive)
     * @param wheelPositionsRad each drive wheel's position in radians of wheel rotation
     * @param gyro the robot's heading
     * @param driveBaseRadius distance from the robot's center to a module, meters
     * @param drive the drivetrain subsystem
     */
    public WheelRadiusCharacterization(
            DoubleConsumer omegaOutput,
            Supplier<double[]> wheelPositionsRad,
            Supplier<Rotation2d> gyro,
            double driveBaseRadius,
            Subsystem drive) {
        this(omegaOutput, wheelPositionsRad, gyro, driveBaseRadius, 1.0, drive);
    }

    /**
     * Creates the characterization.
     *
     * @param omegaOutput takes a robot turn rate in rad/s (counter-clockwise positive)
     * @param wheelPositionsRad each drive wheel's position in radians of wheel rotation
     * @param gyro the robot's heading
     * @param driveBaseRadius distance from the robot's center to a module, meters
     * @param omega how fast to spin, rad/s
     * @param drive the drivetrain subsystem
     */
    public WheelRadiusCharacterization(
            DoubleConsumer omegaOutput,
            Supplier<double[]> wheelPositionsRad,
            Supplier<Rotation2d> gyro,
            double driveBaseRadius,
            double omega,
            Subsystem drive) {
        this.omegaOutput = omegaOutput;
        this.wheelPositionsRad = wheelPositionsRad;
        this.gyro = gyro;
        this.driveBaseRadius = driveBaseRadius;
        this.omega = omega;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        ramp.reset(0);
        startPositions = wheelPositionsRad.get().clone();
        lastGyro = gyro.get();
        gyroAccumulated = 0;
        radius = Double.NaN;
    }

    @Override
    public void execute() {
        omegaOutput.accept(ramp.calculate(omega));

        Rotation2d now = gyro.get();
        gyroAccumulated += Math.abs(now.minus(lastGyro).getRadians());
        lastGyro = now;

        double[] positions = wheelPositionsRad.get();
        double wheelRadians = 0;
        for (int i = 0; i < positions.length; i++) {
            wheelRadians += Math.abs(positions[i] - startPositions[i]) / positions.length;
        }
        // Each wheel travels driveBaseRadius × (robot rotation) along the floor.
        if (wheelRadians > 1e-3) radius = gyroAccumulated * driveBaseRadius / wheelRadians;
    }

    @Override
    public void end(boolean interrupted) {
        omegaOutput.accept(0);
        System.out.printf(
                "[WheelRadiusCharacterization] Wheel radius: %.4f m (%.3f in), robot turned %.1f rad%n",
                radius, radius / 0.0254, gyroAccumulated);
    }

    /** The measured wheel radius in meters, or NaN before the wheels have moved. */
    public double getRadiusMeters() {
        return radius;
    }
}
