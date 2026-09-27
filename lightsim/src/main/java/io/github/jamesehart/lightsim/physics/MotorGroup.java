package io.github.jamesehart.lightsim.physics;

import edu.wpi.first.math.system.plant.DCMotor;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * One or more motors driving the same shaft through a gear reduction.
 *
 * <p>The torque they produce at the output shaft is linear in the output speed ω: {@code τ = A - B·ω},
 * where A comes from the applied voltages and B is the back-EMF damping. Keeping it in this form
 * lets mechanisms integrate it exactly instead of with an unstable explicit step.
 */
public final class MotorGroup {
    private final SimMotor[] motors;
    private final double gearing;
    private final double[] volts;

    /**
     * @param gearing reduction from motor to output (e.g. 6.75 for 6.75:1)
     * @param motors the motors driving the output
     */
    public MotorGroup(double gearing, SimMotor... motors) {
        if (motors.length == 0) throw new IllegalArgumentException("Need at least one motor");
        this.motors = motors.clone();
        this.gearing = gearing;
        this.volts = new double[motors.length];
    }

    public SimMotor[] getMotors() {
        return motors.clone();
    }

    public double getGearing() {
        return gearing;
    }

    /** Reads every motor's applied voltage for this step. */
    public void readVolts(double supplyVolts) {
        for (int i = 0; i < motors.length; i++) {
            volts[i] = motors[i].getAppliedVolts(supplyVolts);
        }
    }

    /** Output torque at zero speed, from the applied voltages (the "A" in τ = A - B·ω). */
    public double driveTorque() {
        double a = 0;
        for (int i = 0; i < motors.length; i++) {
            DCMotor m = motors[i].getMotorModel();
            a += gearing * m.KtNMPerAmp / m.rOhms * volts[i];
        }
        return a;
    }

    /** Back-EMF damping at the output (the "B" in τ = A - B·ω), in N·m per rad/s. */
    public double damping() {
        double b = 0;
        for (SimMotor motor : motors) {
            DCMotor m = motor.getMotorModel();
            b += gearing * gearing * m.KtNMPerAmp / (m.KvRadPerSecPerVolt * m.rOhms);
        }
        return b;
    }

    /** Motor (stator) current of motor {@code i} at the given output speed. */
    public double statorCurrent(int i, double outputRadPerSec) {
        DCMotor m = motors[i].getMotorModel();
        return (volts[i] - gearing * outputRadPerSec / m.KvRadPerSecPerVolt) / m.rOhms;
    }

    /** Total current drawn from the battery at the given output speed. */
    public double supplyCurrent(double outputRadPerSec, double supplyVolts) {
        double total = 0;
        for (int i = 0; i < motors.length; i++) {
            total += Math.abs(statorCurrent(i, outputRadPerSec) * volts[i] / supplyVolts);
        }
        return total;
    }

    /** Writes the output position and speed back to every motor as rotor state. */
    public void writeState(double outputRad, double outputRadPerSec, double supplyVolts, double dt) {
        double rotorRot = outputRad * gearing / (2 * Math.PI);
        double rotorRps = outputRadPerSec * gearing / (2 * Math.PI);
        double mechanismRps = outputRadPerSec / (2 * Math.PI);
        for (int i = 0; i < motors.length; i++) {
            motors[i].setState(
                    rotorRot, rotorRps, mechanismRps, statorCurrent(i, outputRadPerSec), supplyVolts, dt);
        }
    }
}
