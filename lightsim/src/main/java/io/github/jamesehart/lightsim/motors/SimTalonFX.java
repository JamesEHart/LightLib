package io.github.jamesehart.lightsim.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.jamesehart.lightsim.LightSim;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * A {@link TalonFX} that LightSim simulates. On a real robot it is exactly a TalonFX. Defaults to a
 * Kraken X60; pass a {@link DCMotor} for anything else, e.g. {@code DCMotor.getFalcon500(1)}.
 */
public class SimTalonFX extends TalonFX implements SimMotor {
    private final DCMotor model;
    private boolean orientationSet;

    public SimTalonFX(int deviceId) {
        this(deviceId, DCMotor.getKrakenX60(1));
    }

    public SimTalonFX(int deviceId, DCMotor motor) {
        super(deviceId);
        model = motor;
        register();
    }



    public SimTalonFX(int deviceId, CANBus canbus) {
        this(deviceId, canbus, DCMotor.getKrakenX60(1));
    }

    public SimTalonFX(int deviceId, CANBus canbus, DCMotor motor) {
        super(deviceId, canbus);
        model = motor;
        register();
    }

    private void register() {
        if (RobotBase.isSimulation()) LightSim.register(this);
    }

    @Override
    public DCMotor getMotorModel() {
        return model;
    }

    @Override
    public double getAppliedVolts(double supplyVolts) {
        TalonFXSimState sim = getSimState();
        if (!orientationSet) {
            // Simulate in the motor's own positive direction, so inverted motors still move their
            // mechanism the way the code commands.
            MotorOutputConfigs output = new MotorOutputConfigs();
            getConfigurator().refresh(output);
            sim.Orientation = output.Inverted == InvertedValue.Clockwise_Positive
                    ? ChassisReference.Clockwise_Positive
                    : ChassisReference.CounterClockwise_Positive;
            orientationSet = true;
        }
        sim.setSupplyVoltage(supplyVolts);
        return sim.getMotorVoltage();
    }

    @Override
    public void setState(double rotorPositionRot, double rotorVelocityRps, double mechanismVelocityRps,
            double statorCurrentAmps, double supplyVolts, double dtSeconds) {
        TalonFXSimState sim = getSimState();
        sim.setRawRotorPosition(rotorPositionRot);
        sim.setRotorVelocity(rotorVelocityRps);
    }
}
