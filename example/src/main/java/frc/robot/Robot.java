package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import io.github.jamesehart.lightlogger.LightLogger;
import io.github.jamesehart.lightsim.LightSim;
import io.github.jamesehart.lightsim.motors.SimPigeon2;
import io.github.jamesehart.lightsim.motors.SimSparkMax;
import io.github.jamesehart.lightsim.motors.SimTalonFX;
import io.github.jamesehart.lightsim.physics.SwerveSimConfig;

public class Robot extends TimedRobot {
    private static final double DRIVE_GEARING = 6.75;
    private static final double STEER_GEARING = 150.0 / 7;
    private static final double WHEEL_RADIUS = 0.0508;
    private static final double MAX_SPEED = 4.5;
    private static final double MAX_TURN = 2 * Math.PI;

    private static final double ELEVATOR_GEARING = 10;
    private static final double ELEVATOR_DRUM_RADIUS = 0.03;

    private final PIDController pid = new PIDController(0.1, 0, 0);

    private final Translation2d[] locations = {
        new Translation2d(0.28, 0.28), new Translation2d(0.28, -0.28),
        new Translation2d(-0.28, 0.28), new Translation2d(-0.28, -0.28)
    };
    private final SimTalonFX[] drives = new SimTalonFX[4];
    private final SimTalonFX[] steers = new SimTalonFX[4];
    private final SimPigeon2 pigeon = new SimPigeon2(30);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locations);
    private final SwerveDriveOdometry odometry;

    private final SimSparkMax flywheel = new SimSparkMax(20, MotorType.kBrushless);
    private final SimTalonFX elevator = new SimTalonFX(15);

    private final XboxController controller = new XboxController(0);
    private final VoltageOut driveRequest = new VoltageOut(0);
    private final PositionVoltage steerRequest = new PositionVoltage(0);
    private final PositionVoltage elevatorRequest = new PositionVoltage(0);

    public Robot() {
        LightSim.enable(this);
        LightLogger.start(false);

        // Style 2: register once, the callback runs whenever the value is edited
        LightLogger.tunableNumber("Tuning/kP", 0.1, pid::setP);

        // Swerve: TalonFX drive and steer motors, a Pigeon2, and the matching LightSim config.
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.Slot0.kP = 40;
        steerConfig.Feedback.SensorToMechanismRatio = STEER_GEARING;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        SwerveSimConfig swerveSim = new SwerveSimConfig()
                .driveGearing(DRIVE_GEARING)
                .steerGearing(STEER_GEARING)
                .wheelRadius(WHEEL_RADIUS)
                .robotMass(55)
                .bumperSize(0.9, 0.9)
                .gyro(pigeon);
        for (int i = 0; i < 4; i++) {
            drives[i] = new SimTalonFX(1 + i * 2);
            steers[i] = new SimTalonFX(2 + i * 2);
            steers[i].getConfigurator().apply(steerConfig);
            swerveSim.module(drives[i], steers[i], locations[i]);
        }
        LightSim.swerve(swerveSim);
        Pose2d start = new Pose2d(2, 4, Rotation2d.kZero);
        LightSim.setRobotPose(start);
        odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(), getModulePositions(), start);

        // A flywheel on a NEO, and an elevator shown as component 0 of an AdvantageScope robot model.
        LightSim.flywheel(flywheel, 1, 0.004);

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.Slot0.kP = 60;
        elevatorConfig.Slot0.kG = 0.3;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Feedback.SensorToMechanismRatio = ELEVATOR_GEARING / (2 * Math.PI * ELEVATOR_DRUM_RADIUS);
        elevator.getConfigurator().apply(elevatorConfig);
        LightSim.elevator(elevator, ELEVATOR_GEARING, 8, ELEVATOR_DRUM_RADIUS, 0, 1.2)
                .component(0, new Translation3d(0, 0, 0.1));
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            double meters = drives[i].getPosition().getValueAsDouble() / DRIVE_GEARING * 2 * Math.PI * WHEEL_RADIUS;
            positions[i] = new SwerveModulePosition(meters, getAngle(i));
        }
        return positions;
    }

    private Rotation2d getAngle(int i) {
        return Rotation2d.fromRotations(steers[i].getPosition().getValueAsDouble());
    }

    @Override
    public void robotPeriodic() {
        LightLogger.startFrame();
        LightLogger.logNumber("Time", Timer.getFPGATimestamp());

        // Style 1: read the current value every loop
        double speed = LightLogger.tunableNumber("Tuning/Speed", 0.5);
        LightLogger.logNumber("SpeedTimesKP", speed * pid.getP());

        // Compare the robot's own odometry with LightSim's true pose (/LightSim/RobotPose).
        LightLogger.logPose2d("Odometry", odometry.update(pigeon.getRotation2d(), getModulePositions()));
        LightLogger.logNumber("Flywheel/RPM", flywheel.getEncoder().getVelocity());
        LightLogger.logNumber("Elevator/HeightM", elevator.getPosition().getValueAsDouble());
        LightLogger.endFrame();
    }

    @Override
    public void teleopPeriodic() {
        // Field-relative swerve: left stick drives, right stick X turns.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -controller.getLeftY() * MAX_SPEED,
                -controller.getLeftX() * MAX_SPEED,
                -controller.getRightX() * MAX_TURN,
                pigeon.getRotation2d());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
        for (int i = 0; i < 4; i++) {
            states[i].optimize(getAngle(i));
            drives[i].setControl(driveRequest.withOutput(states[i].speedMetersPerSecond / MAX_SPEED * 12));
            if (Math.abs(states[i].speedMetersPerSecond) > 0.01) {
                steers[i].setControl(steerRequest.withPosition(states[i].angle.getRotations()));
            }
        }

        // Right trigger spins the flywheel, A raises the elevator, B lowers it.
        flywheel.setVoltage(controller.getRightTriggerAxis() * 12);
        if (controller.getAButtonPressed()) elevator.setControl(elevatorRequest.withPosition(1.0));
        if (controller.getBButtonPressed()) elevator.setControl(elevatorRequest.withPosition(0));
    }
}
