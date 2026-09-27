package io.github.jamesehart.lightsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.jamesehart.lightsim.SimGyro;
import io.github.jamesehart.lightsim.SimMotor;

class PhysicsTest {
    private static final double DT = 0.02;
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.07;

    /** A motor with a fixed applied voltage that records what LightSim writes back. */
    static final class FakeMotor implements SimMotor {
        final DCMotor model;
        double volts;
        double rotorPositionRot;
        double rotorVelocityRps;
        double currentAmps;

        FakeMotor(DCMotor model, double volts) {
            this.model = model;
            this.volts = volts;
        }

        @Override
        public DCMotor getMotorModel() {
            return model;
        }

        @Override
        public double getAppliedVolts(double supplyVolts) {
            return volts;
        }

        @Override
        public void setState(double pos, double vel, double mechVel, double amps, double supply, double dt) {
            rotorPositionRot = pos;
            rotorVelocityRps = vel;
            currentAmps = amps;
        }
    }

    static final class FakeGyro implements SimGyro {
        double yawDeg;
        double rateDps;

        @Override
        public void setSimulatedYaw(double yawDegrees, double rate) {
            yawDeg = yawDegrees;
            rateDps = rate;
        }
    }

    private static void run(io.github.jamesehart.lightsim.SimLoad load, double seconds) {
        for (int i = 0; i < Math.round(seconds / DT); i++) {
            load.step(DT, 12);
        }
    }

    private static FakeMotor kraken(double volts) {
        return new FakeMotor(DCMotor.getKrakenX60(1), volts);
    }

    @Test
    void flywheelReachesFreeSpeed() {
        FakeMotor motor = kraken(12);
        RotaryMechanism flywheel = RotaryMechanism.flywheel(new MotorGroup(2, motor), 0.004);
        run(flywheel, 3);

        double expected = DCMotor.getKrakenX60(1).freeSpeedRadPerSec / 2;
        assertEquals(expected, flywheel.getVelocity(), expected * 0.02);
        assertEquals(flywheel.getVelocity() * 2 / (2 * Math.PI), motor.rotorVelocityRps, 1e-6);
        assertTrue(motor.rotorPositionRot > 0);
    }

    @Test
    void armFallsToLowerStopWithNoPower() {
        FakeMotor motor = kraken(0);
        RotaryMechanism arm = RotaryMechanism.arm(new MotorGroup(20, motor), 0.5, 0.6, -0.5, 1.5);
        motor.volts = 6;
        run(arm, 1);
        assertTrue(arm.getPosition() > 0, "arm should lift under power");
        motor.volts = 0;
        run(arm, 5);
        assertEquals(-0.5, arm.getPosition(), 1e-9);
    }

    @Test
    void elevatorStopsAtTop() {
        FakeMotor motor = kraken(12);
        RotaryMechanism elevator = RotaryMechanism.elevator(new MotorGroup(10, motor), 8, 0.03, 0, 1.2);
        run(elevator, 3);
        assertEquals(1.2, elevator.getPosition(), 1e-9);
    }

    private static DrivetrainSim swerve(FakeMotor[] drives, FakeMotor[] steers, FakeGyro gyro) {
        SwerveSimConfig config = new SwerveSimConfig().gyro(gyro);
        double[][] locations = {{0.28, 0.28}, {0.28, -0.28}, {-0.28, 0.28}, {-0.28, -0.28}};
        for (int i = 0; i < 4; i++) {
            config.module(drives[i], steers[i], new Translation2d(locations[i][0], locations[i][1]));
        }
        return DrivetrainSim.swerve(new PhysicsWorld(FIELD_LENGTH, FIELD_WIDTH), config);
    }

    private static FakeMotor[] krakens(double volts) {
        return new FakeMotor[] {kraken(volts), kraken(volts), kraken(volts), kraken(volts)};
    }

    @Test
    void swerveDrivesStraightAtTopSpeed() {
        FakeMotor[] drives = krakens(12);
        DrivetrainSim drive = swerve(drives, krakens(0), new FakeGyro());
        drive.setPose(new Pose2d(1.5, 4, Rotation2d.kZero));
        run(drive, 2);

        Pose2d start = drive.getPose();
        run(drive, 0.5);
        double speed = (drive.getPose().getX() - start.getX()) / 0.5;
        double topSpeed = DCMotor.getKrakenX60(1).freeSpeedRadPerSec / 6.75 * 0.0508;
        assertEquals(topSpeed, speed, topSpeed * 0.05);
        assertEquals(4, drive.getPose().getY(), 0.01);
        assertEquals(0, drive.getPose().getRotation().getDegrees(), 0.5);
        assertTrue(drives[0].rotorPositionRot > 0);
    }

    @Test
    void accelerationIsLimitedByTraction() {
        FakeMotor[] drives = krakens(12);
        DrivetrainSim drive = swerve(drives, krakens(0), new FakeGyro());
        drive.setPose(new Pose2d(1.5, 4, Rotation2d.kZero));
        run(drive, 0.1);
        // Starting from rest: x = a·t²/2, and a can't exceed μ·g.
        double accel = 2 * (drive.getPose().getX() - 1.5) / (0.1 * 0.1);
        assertTrue(accel <= 1.2 * 9.81 * 1.05, "acceleration " + accel + " is more than traction allows");
        assertTrue(accel > 5, "acceleration " + accel + " is too low");
    }

    @Test
    void wallStopsTheRobot() {
        DrivetrainSim drive = swerve(krakens(12), krakens(0), new FakeGyro());
        drive.setPose(new Pose2d(FIELD_LENGTH - 2, 4, Rotation2d.kZero));
        run(drive, 3);
        assertTrue(drive.getPose().getX() <= FIELD_LENGTH - 0.45 + 0.02, "x = " + drive.getPose().getX());
        assertTrue(drive.getPose().getX() > FIELD_LENGTH - 0.6);
    }

    @Test
    void steeringTurnsModules() {
        FakeMotor[] steers = krakens(2);
        DrivetrainSim drive = swerve(krakens(0), steers, new FakeGyro());
        run(drive, 0.5);
        assertTrue(drive.getModuleStates()[0].angle.getRadians() > 0.1);
    }

    @Test
    void tankSpinsInPlace() {
        FakeGyro gyro = new FakeGyro();
        FakeMotor left = kraken(-6);
        FakeMotor right = kraken(6);
        DrivetrainSim drive = DrivetrainSim.tank(
                new PhysicsWorld(FIELD_LENGTH, FIELD_WIDTH), new TankSimConfig().left(left).right(right).gyro(gyro));
        Pose2d start = drive.getPose();
        run(drive, 2);

        assertTrue(gyro.rateDps > 30, "should spin counter-clockwise, rate " + gyro.rateDps);
        assertTrue(gyro.yawDeg > 60);
        assertEquals(0, drive.getPose().getTranslation().getDistance(start.getTranslation()), 0.05);
    }

    @Test
    void stopsWhenPowerIsCut() {
        FakeMotor[] drives = krakens(12);
        DrivetrainSim drive = swerve(drives, krakens(0), new FakeGyro());
        drive.setPose(new Pose2d(3, 4, Rotation2d.kZero));
        run(drive, 1);
        for (FakeMotor m : drives) m.volts = 0;
        run(drive, 1);
        Pose2d stopped = drive.getPose();
        run(drive, 0.5);
        assertEquals(stopped.getX(), drive.getPose().getX(), 0.01);
    }
}
