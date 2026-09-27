package io.github.jamesehart.lightsim;

/** Something driven by simulated motors: a mechanism or a drivetrain. */
public interface SimLoad {
    /**
     * Advances the simulation and writes the new state back into the motors.
     *
     * @param dtSeconds time step
     * @param supplyVolts battery voltage
     */
    void step(double dtSeconds, double supplyVolts);

    /** Total current drawn from the battery during the last step, in amps. */
    double getSupplyCurrentAmps();
}
