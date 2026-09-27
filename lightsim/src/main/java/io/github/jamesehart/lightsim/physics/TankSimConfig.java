package io.github.jamesehart.lightsim.physics;

import io.github.jamesehart.lightsim.SimGyro;
import io.github.jamesehart.lightsim.SimMotor;

/** Describes a tank (differential) drivetrain for {@code LightSim.tank(...)}. Every setter returns this. */
public final class TankSimConfig {
    SimMotor[] left = new SimMotor[0];
    SimMotor[] right = new SimMotor[0];
    double trackWidthM = 0.6;
    double gearing = 8.45;
    double wheelRadiusM = 0.0762;
    double robotMassKg = 50;
    double bumperLengthM = 0.9;
    double bumperWidthM = 0.9;
    double wheelCof = 1.2;
    SimGyro gyro;

    /** Left side motors. Positive rotation must drive forward, as on the real robot after inversion. */
    public TankSimConfig left(SimMotor... motors) {
        left = motors.clone();
        return this;
    }

    /** Right side motors. Positive rotation must drive forward, as on the real robot after inversion. */
    public TankSimConfig right(SimMotor... motors) {
        right = motors.clone();
        return this;
    }

    /** Distance between the left and right wheels, in meters. Default 0.6. */
    public TankSimConfig trackWidth(double meters) {
        trackWidthM = meters;
        return this;
    }

    /** Drive reduction. Default 8.45 (KitBot). */
    public TankSimConfig gearing(double gearing) {
        this.gearing = gearing;
        return this;
    }

    /** Wheel radius in meters. Default 0.0762 (6 in wheel). */
    public TankSimConfig wheelRadius(double meters) {
        wheelRadiusM = meters;
        return this;
    }

    /** Robot mass with bumpers and battery, in kg. Default 50. */
    public TankSimConfig robotMass(double kg) {
        robotMassKg = kg;
        return this;
    }

    /** Outer bumper size in meters (length along x, width along y). Default 0.9 × 0.9. */
    public TankSimConfig bumperSize(double lengthM, double widthM) {
        bumperLengthM = lengthM;
        bumperWidthM = widthM;
        return this;
    }

    /** Wheel-to-carpet coefficient of friction. Default 1.2. */
    public TankSimConfig wheelCOF(double cof) {
        wheelCof = cof;
        return this;
    }

    /** Gyro to update with the simulated heading (optional). */
    public TankSimConfig gyro(SimGyro gyro) {
        this.gyro = gyro;
        return this;
    }
}
