package io.github.jamesehart.lightsim.physics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import io.github.jamesehart.lightsim.SimGyro;
import io.github.jamesehart.lightsim.SimMotor;

/** Describes a swerve drivetrain for {@code LightSim.swerve(...)}. Every setter returns this. */
public final class SwerveSimConfig {
    record Module(SimMotor drive, SimMotor steer, Translation2d location) {}

    final List<Module> modules = new ArrayList<>();
    double driveGearing = 6.75;
    double steerGearing = 150.0 / 7;
    double wheelRadiusM = 0.0508;
    double robotMassKg = 55;
    double bumperLengthM = 0.9;
    double bumperWidthM = 0.9;
    double wheelCof = 1.2;
    double steerMoiKgM2 = 0.004;
    SimGyro gyro;

    /**
     * Adds a module. Positive steer rotation must turn the module counter-clockwise, and positive
     * drive rotation must drive it forward, as on the real robot after inversion is applied.
     *
     * @param drive drive motor
     * @param steer steering motor
     * @param location module position relative to robot center (x forward, y left), meters
     */
    public SwerveSimConfig module(SimMotor drive, SimMotor steer, Translation2d location) {
        modules.add(new Module(drive, steer, location));
        return this;
    }

    /** Drive reduction, e.g. 6.75 for SDS MK4i L2. Default 6.75. */
    public SwerveSimConfig driveGearing(double gearing) {
        driveGearing = gearing;
        return this;
    }

    /** Steering reduction, e.g. 150/7 for SDS MK4i. Default 150/7. */
    public SwerveSimConfig steerGearing(double gearing) {
        steerGearing = gearing;
        return this;
    }

    /** Wheel radius in meters. Default 0.0508 (4 in wheel). */
    public SwerveSimConfig wheelRadius(double meters) {
        wheelRadiusM = meters;
        return this;
    }

    /** Robot mass with bumpers and battery, in kg. Default 55. */
    public SwerveSimConfig robotMass(double kg) {
        robotMassKg = kg;
        return this;
    }

    /** Outer bumper size in meters (length along x, width along y). Default 0.9 × 0.9. */
    public SwerveSimConfig bumperSize(double lengthM, double widthM) {
        bumperLengthM = lengthM;
        bumperWidthM = widthM;
        return this;
    }

    /** Wheel-to-carpet coefficient of friction. Default 1.2. */
    public SwerveSimConfig wheelCOF(double cof) {
        wheelCof = cof;
        return this;
    }

    /** Moment of inertia of a module's steering, in kg·m². Default 0.004. */
    public SwerveSimConfig steerMOI(double kgM2) {
        steerMoiKgM2 = kgM2;
        return this;
    }

    /** Gyro to update with the simulated heading (optional). */
    public SwerveSimConfig gyro(SimGyro gyro) {
        this.gyro = gyro;
        return this;
    }
}
