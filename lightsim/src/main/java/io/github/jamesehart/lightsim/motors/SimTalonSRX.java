package io.github.jamesehart.lightsim.motors;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.jamesehart.lightsim.LightSim;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * A {@link WPI_TalonSRX} (Phoenix 5) that LightSim simulates. On a real robot it is exactly a
 * WPI_TalonSRX. Pass the motor it runs, e.g. {@code DCMotor.getCIM(1)}.
 *
 * <p>The simulated quadrature encoder defaults to 4096 counts per rotation (CTRE Mag Encoder) on
 * the motor shaft. If the motor is inverted, set the sensor phase like on the real robot.
 */
public class SimTalonSRX extends WPI_TalonSRX implements SimMotor {
    private final DCMotor model;
    private double countsPerRotation = 4096;

    public SimTalonSRX(int deviceId, DCMotor motor) {
        super(deviceId);
        model = motor;
        if (RobotBase.isSimulation()) LightSim.register(this);
    }

    /**
     * Sets the simulated encoder resolution.
     *
     * @param counts quadrature counts per motor rotation
     * @return this, for chaining
     */
    public SimTalonSRX withEncoderCountsPerRotation(double counts) {
        countsPerRotation = counts;
        return this;
    }

    @Override
    public DCMotor getMotorModel() {
        return model;
    }

    private double direction() {
        return getInverted() ? -1 : 1;
    }

    @Override
    public double getAppliedVolts(double supplyVolts) {
        TalonSRXSimCollection sim = getSimCollection();
        sim.setBusVoltage(supplyVolts);
        return sim.getMotorOutputLeadVoltage() * direction();
    }

    @Override
    public void setState(double rotorPositionRot, double rotorVelocityRps, double mechanismVelocityRps,
            double statorCurrentAmps, double supplyVolts, double dtSeconds) {
        TalonSRXSimCollection sim = getSimCollection();
        // The lead voltage (and a real encoder on the shaft) follow the motor's wires, not the
        // inverted direction, so undo the inversion here.
        sim.setQuadratureRawPosition((int) Math.round(rotorPositionRot * countsPerRotation * direction()));
        sim.setQuadratureVelocity((int) Math.round(rotorVelocityRps * countsPerRotation / 10 * direction()));
        double duty = Math.abs(sim.getMotorOutputLeadVoltage() / supplyVolts);
        sim.setStatorCurrent(Math.abs(statorCurrentAmps));
        sim.setSupplyCurrent(Math.abs(statorCurrentAmps) * duty);
    }
}
