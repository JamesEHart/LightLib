package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import io.github.jamesehart.lightlogger.LightLogger;

public class Robot extends TimedRobot {

    private final PIDController pid = new PIDController(0.1, 0, 0);

    public Robot() {
        LightLogger.start(false);

        // Style 2: register once, the callback runs whenever the value is edited
        LightLogger.tunableNumber("Tuning/kP", 0.1, pid::setP);
    }

    @Override
    public void robotPeriodic() {
        LightLogger.startFrame();
        LightLogger.logNumber("Time", Timer.getFPGATimestamp());

        // Style 1: read the current value every loop
        double speed = LightLogger.tunableNumber("Tuning/Speed", 0.5);
        LightLogger.logNumber("SpeedTimesKP", speed * pid.getP());
        LightLogger.endFrame();
    }

    @Override
    public void teleopPeriodic() {
        LightLogger.logPose2d("Pose2dTest", new Pose2d(new Translation2d(10, 5), Rotation2d.fromDegrees(45)));
    }
}
