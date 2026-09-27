package io.github.jamesehart.lightsim.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.jamesehart.lightsim.LightSim;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * A {@link TalonFXS} that LightSim simulates. On a real robot it is exactly a TalonFXS. The FXS runs
 * many motor types, so pass the one you use, e.g. {@code DCMotor.getMinion(1)} or {@code
 * DCMotor.getNEO(1)}.
 */
public class SimTalonFXS extends TalonFXS implements SimMotor {
    private final DCMotor model;
    private boolean orientationSet;

    public SimTalonFXS(int deviceId, DCMotor motor) {
        super(deviceId);
        model = motor;
        register();
    }


    public SimTalonFXS(int deviceId, CANBus canbus, DCMotor motor) {
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
        TalonFXSSimState sim = getSimState();
        if (!orientationSet) {
            // Simulate in the motor's own positive direction, so inverted motors still move their
            // mechanism the way the code commands.
            MotorOutputConfigs output = new MotorOutputConfigs();
            getConfigurator().refresh(output);
            sim.MotorOrientation = output.Inverted == InvertedValue.Clockwise_Positive
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
        TalonFXSSimState sim = getSimState();
        sim.setRawRotorPosition(rotorPositionRot);
        sim.setRotorVelocity(rotorVelocityRps);
    }
}
