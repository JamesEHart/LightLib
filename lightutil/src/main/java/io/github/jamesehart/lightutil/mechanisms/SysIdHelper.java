package io.github.jamesehart.lightutil.mechanisms;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Sets up WPILib's SysId characterization for a mechanism in one line, from plain numbers instead
 * of units and log callbacks. SysId measures kS, kV, kA (and kG) so you don't have to guess your
 * feedforward gains.
 *
 * <pre>{@code
 * SysIdRoutine shooterId = SysIdHelper.rotary("Shooter",
 *     shooter::setVoltage,                                  // volts
 *     () -> motor.getPosition().getValueAsDouble(),         // rotations
 *     () -> motor.getVelocity().getValueAsDouble(),         // rotations per second
 *     shooter);
 * SysIdHelper.bind(shooterId, controller.a(), controller.b(), controller.x(), controller.y());
 * // or run everything in one go:
 * controller.start().onTrue(SysIdHelper.runAll(shooterId));
 * }</pre>
 *
 * <p>Then open the log file (from the roboRIO, or {@code logs/} in sim) in the SysId tool. Run each
 * test until the mechanism nears its limit, then let go. Creating a routine starts {@code
 * DataLogManager}, because SysId data is only recorded to its log file.
 *
 * <p><b>CTRE users:</b> Phoenix 6's SignalLogger records at a higher rate; you can still use these
 * routines to drive the tests.
 */
public final class SysIdHelper {
    private SysIdHelper() {}

    /**
     * A routine for a spinning mechanism (flywheel, arm, turret, swerve steer), with WPILib's
     * default test settings (1 V/s ramp, 7 V step, 10 s timeout).
     *
     * @param name name in the log, e.g. "Shooter"
     * @param voltage sets the motor voltage
     * @param positionRotations mechanism position, rotations
     * @param velocityRps mechanism velocity, rotations per second
     * @param mechanism the subsystem
     * @return the routine
     */
    public static SysIdRoutine rotary(
            String name, DoubleConsumer voltage, DoubleSupplier positionRotations, DoubleSupplier velocityRps,
            Subsystem mechanism) {
        return rotary(name, voltage, positionRotations, velocityRps, mechanism, 1, 7, 10);
    }

    /**
     * A routine for a spinning mechanism with custom test settings. Use a lower step voltage for
     * mechanisms with little travel, like arms.
     *
     * @param name name in the log
     * @param voltage sets the motor voltage
     * @param positionRotations mechanism position, rotations
     * @param velocityRps mechanism velocity, rotations per second
     * @param mechanism the subsystem
     * @param rampVoltsPerSecond quasistatic ramp rate
     * @param stepVolts dynamic step voltage
     * @param timeoutSeconds each test stops after this long
     * @return the routine
     */
    public static SysIdRoutine rotary(
            String name, DoubleConsumer voltage, DoubleSupplier positionRotations, DoubleSupplier velocityRps,
            Subsystem mechanism, double rampVoltsPerSecond, double stepVolts, double timeoutSeconds) {
        double[] applied = new double[1];
        DataLogManager.start();
        return new SysIdRoutine(
                config(rampVoltsPerSecond, stepVolts, timeoutSeconds),
                new SysIdRoutine.Mechanism(
                        volts -> {
                            applied[0] = volts.in(Volts);
                            voltage.accept(applied[0]);
                        },
                        log -> log.motor(name)
                                .voltage(Volts.of(applied[0]))
                                .angularPosition(Rotations.of(positionRotations.getAsDouble()))
                                .angularVelocity(RotationsPerSecond.of(velocityRps.getAsDouble())),
                        mechanism,
                        name));
    }

    /**
     * A routine for a mechanism that moves in a line (elevator, drivetrain driving straight), with
     * WPILib's default test settings.
     *
     * @param name name in the log
     * @param voltage sets the motor voltage
     * @param positionMeters position, meters
     * @param velocityMps velocity, meters per second
     * @param mechanism the subsystem
     * @return the routine
     */
    public static SysIdRoutine linear(
            String name, DoubleConsumer voltage, DoubleSupplier positionMeters, DoubleSupplier velocityMps,
            Subsystem mechanism) {
        return linear(name, voltage, positionMeters, velocityMps, mechanism, 1, 7, 10);
    }

    /**
     * A routine for a mechanism that moves in a line, with custom test settings. Use a lower step
     * voltage for elevators, so they don't slam into the top.
     *
     * @param name name in the log
     * @param voltage sets the motor voltage
     * @param positionMeters position, meters
     * @param velocityMps velocity, meters per second
     * @param mechanism the subsystem
     * @param rampVoltsPerSecond quasistatic ramp rate
     * @param stepVolts dynamic step voltage
     * @param timeoutSeconds each test stops after this long
     * @return the routine
     */
    public static SysIdRoutine linear(
            String name, DoubleConsumer voltage, DoubleSupplier positionMeters, DoubleSupplier velocityMps,
            Subsystem mechanism, double rampVoltsPerSecond, double stepVolts, double timeoutSeconds) {
        double[] applied = new double[1];
        DataLogManager.start();
        return new SysIdRoutine(
                config(rampVoltsPerSecond, stepVolts, timeoutSeconds),
                new SysIdRoutine.Mechanism(
                        volts -> {
                            applied[0] = volts.in(Volts);
                            voltage.accept(applied[0]);
                        },
                        log -> log.motor(name)
                                .voltage(Volts.of(applied[0]))
                                .linearPosition(Meters.of(positionMeters.getAsDouble()))
                                .linearVelocity(MetersPerSecond.of(velocityMps.getAsDouble())),
                        mechanism,
                        name));
    }

    private static SysIdRoutine.Config config(double rampVoltsPerSecond, double stepVolts, double timeoutSeconds) {
        return new SysIdRoutine.Config(
                Volts.of(rampVoltsPerSecond).per(Second), Volts.of(stepVolts), Seconds.of(timeoutSeconds));
    }

    /**
     * Binds the four tests to buttons. Each runs while its button is held, so let go before the
     * mechanism hits a limit.
     *
     * @param routine the routine
     * @param quasistaticForward e.g. {@code controller.a()}
     * @param quasistaticReverse e.g. {@code controller.b()}
     * @param dynamicForward e.g. {@code controller.x()}
     * @param dynamicReverse e.g. {@code controller.y()}
     */
    public static void bind(
            SysIdRoutine routine,
            Trigger quasistaticForward,
            Trigger quasistaticReverse,
            Trigger dynamicForward,
            Trigger dynamicReverse) {
        quasistaticForward.whileTrue(routine.quasistatic(Direction.kForward));
        quasistaticReverse.whileTrue(routine.quasistatic(Direction.kReverse));
        dynamicForward.whileTrue(routine.dynamic(Direction.kForward));
        dynamicReverse.whileTrue(routine.dynamic(Direction.kReverse));
    }

    /**
     * Runs all four tests back to back, with a pause between them for the mechanism to stop. Only
     * for mechanisms that can move freely for the whole test, like flywheels and drivetrains with
     * room to drive; each test runs to its timeout.
     *
     * @param routine the routine
     * @return the command
     */
    public static Command runAll(SysIdRoutine routine) {
        return Commands.sequence(
                        routine.quasistatic(Direction.kForward),
                        Commands.waitSeconds(1.5),
                        routine.quasistatic(Direction.kReverse),
                        Commands.waitSeconds(1.5),
                        routine.dynamic(Direction.kForward),
                        Commands.waitSeconds(1.5),
                        routine.dynamic(Direction.kReverse))
                .withName("SysIdAll");
    }
}
