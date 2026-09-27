package io.github.jamesehart.lightsim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A motor controller that LightSim can simulate. The {@code Sim*} classes (e.g. {@code
 * SimTalonFX}) implement this; you only need it yourself to add support for another controller.
 *
 * <p>Everything is in the motor's own positive direction: positive applied voltage moves the rotor
 * positive, the same way it does on the real robot after inversion is applied.
 */
public interface SimMotor {
    /** The physical motor model, e.g. {@code DCMotor.getKrakenX60(1)}. */
    DCMotor getMotorModel();

    /**
     * Sets the supply (battery) voltage and returns the voltage the controller is currently
     * applying to the motor.
     *
     * @param supplyVolts battery voltage
     * @return applied motor voltage
     */
    double getAppliedVolts(double supplyVolts);

    /**
     * Writes the simulated state back into the controller's sensors.
     *
     * @param rotorPositionRot rotor position in rotations
     * @param rotorVelocityRps rotor velocity in rotations per second
     * @param mechanismVelocityRps mechanism (after gearing) velocity in rotations per second
     * @param statorCurrentAmps motor current
     * @param supplyVolts battery voltage
     * @param dtSeconds time since the last update
     */
    void setState(
            double rotorPositionRot,
            double rotorVelocityRps,
            double mechanismVelocityRps,
            double statorCurrentAmps,
            double supplyVolts,
            double dtSeconds);
}
