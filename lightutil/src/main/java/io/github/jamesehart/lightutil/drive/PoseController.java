package io.github.jamesehart.lightutil.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Drives a holonomic (swerve/mecanum) robot to a pose in a straight line, with motion profiles on
 * both distance and heading. Use it for auto-aligning to a scoring location, the last bit of an
 * auto path, or snapping to an angle.
 *
 * <pre>{@code
 * PoseController align = new PoseController();
 * controller.a().whileTrue(align.driveTo(
 *     drive::getPose,
 *     () -> GeomUtil.nearest(drive.getPose(), scoringPoses),
 *     drive::driveRobotRelative,
 *     drive));
 * }</pre>
 */
public final class PoseController {
    private final ProfiledPIDController distanceController;
    private final ProfiledPIDController headingController;
    private double positionTolerance = 0.02;
    private double angleTolerance = Math.toRadians(2);
    private boolean atGoal = false;

    /** Creates a controller with gentle default gains and limits (3 m/s, 3 m/s², 2π rad/s, 4π rad/s²). */
    public PoseController() {
        this(4, 3, 3, 5, 2 * Math.PI, 4 * Math.PI);
    }

    /**
     * Creates a controller.
     *
     * @param translationKp P gain on distance error (m/s per meter)
     * @param maxSpeed m/s
     * @param maxAcceleration m/s²
     * @param headingKp P gain on heading error (rad/s per radian)
     * @param maxTurnRate rad/s
     * @param maxTurnAcceleration rad/s²
     */
    public PoseController(
            double translationKp,
            double maxSpeed,
            double maxAcceleration,
            double headingKp,
            double maxTurnRate,
            double maxTurnAcceleration) {
        distanceController = new ProfiledPIDController(
                translationKp, 0, 0, new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));
        headingController = new ProfiledPIDController(
                headingKp, 0, 0, new TrapezoidProfile.Constraints(maxTurnRate, maxTurnAcceleration));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets how close counts as arrived. Defaults are 2 cm and 2°.
     *
     * @param meters position tolerance
     * @param radians heading tolerance
     * @return this, for chaining
     */
    public PoseController withTolerance(double meters, double radians) {
        this.positionTolerance = meters;
        this.angleTolerance = radians;
        return this;
    }

    /**
     * Resets the profiles to start from the robot's current state. Call when you start driving to
     * a new goal ({@link #driveTo} does this for you).
     *
     * @param current robot pose
     * @param goal the pose to drive to
     * @param fieldSpeeds the robot's current field-relative speeds
     */
    public void reset(Pose2d current, Pose2d goal, ChassisSpeeds fieldSpeeds) {
        Translation2d offset = current.getTranslation().minus(goal.getTranslation());
        double distance = offset.getNorm();
        // Speed along the line toward the goal (negative distance rate while approaching).
        double approachSpeed = distance < 1e-6 ? 0
                : (fieldSpeeds.vxMetersPerSecond * offset.getX() + fieldSpeeds.vyMetersPerSecond * offset.getY()) / distance;
        distanceController.reset(distance, approachSpeed);
        headingController.reset(current.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);
        atGoal = false;
    }

    /**
     * Calculates the field-relative speeds to drive toward the goal. Call every loop.
     *
     * @param current robot pose
     * @param goal the pose to drive to
     * @return field-relative speeds
     */
    public ChassisSpeeds calculate(Pose2d current, Pose2d goal) {
        Translation2d offset = current.getTranslation().minus(goal.getTranslation());
        double distance = offset.getNorm();

        double speed = distanceController.calculate(distance, 0) + distanceController.getSetpoint().velocity;
        if (distance < positionTolerance) speed = 0;
        Translation2d velocity = distance < 1e-6 ? Translation2d.kZero : offset.div(distance).times(speed);

        double headingError = current.getRotation().minus(goal.getRotation()).getRadians();
        double omega = headingController.calculate(current.getRotation().getRadians(), goal.getRotation().getRadians())
                + headingController.getSetpoint().velocity;
        if (Math.abs(headingError) < angleTolerance) omega = 0;

        atGoal = distance < positionTolerance && Math.abs(headingError) < angleTolerance;
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    }

    /** True if the last {@link #calculate} call was within tolerance of the goal. */
    public boolean atGoal() {
        return atGoal;
    }

    /**
     * A command that drives to a pose and ends when it gets there.
     *
     * @param pose the robot's current pose
     * @param goal the pose to drive to; read once when the command starts
     * @param robotRelativeOutput takes robot-relative speeds, e.g. {@code drive::driveRobotRelative}
     * @param drive the drivetrain subsystem
     * @return the command
     */
    public Command driveTo(
            Supplier<Pose2d> pose,
            Supplier<Pose2d> goal,
            Consumer<ChassisSpeeds> robotRelativeOutput,
            Subsystem drive) {
        Pose2d[] target = new Pose2d[1];
        return new FunctionalCommand(
                () -> {
                    target[0] = goal.get();
                    reset(pose.get(), target[0], new ChassisSpeeds());
                },
                () -> {
                    Pose2d current = pose.get();
                    robotRelativeOutput.accept(
                            ChassisSpeeds.fromFieldRelativeSpeeds(calculate(current, target[0]), current.getRotation()));
                },
                interrupted -> robotRelativeOutput.accept(new ChassisSpeeds()),
                this::atGoal,
                drive);
    }
}
