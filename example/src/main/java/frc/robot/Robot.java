package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import io.github.jamesehart.lightlogger.LightLogger;

public class Robot extends TimedRobot {

    public Robot() {
        LightLogger.start(true);
    }

    @Override
    public void robotPeriodic() {
        LightLogger.startFrame();
        LightLogger.logNumber("Time", Timer.getFPGATimestamp());
        LightLogger.endFrame();
    }

    @Override
    public void teleopPeriodic() {
        LightLogger.logPose2d("Pose2dTest", new Pose2d(new Translation2d(10, 5), Rotation2d.fromDegrees(45)));
    }
}
