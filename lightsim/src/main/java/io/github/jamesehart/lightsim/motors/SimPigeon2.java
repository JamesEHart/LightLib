package io.github.jamesehart.lightsim.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import io.github.jamesehart.lightsim.SimGyro;

/**
 * A {@link Pigeon2} whose yaw follows the simulated drivetrain. On a real robot it is exactly a
 * Pigeon2. Pass it to the drivetrain config with {@code .gyro(pigeon)}.
 */
public class SimPigeon2 extends Pigeon2 implements SimGyro {
    public SimPigeon2(int deviceId) {
        super(deviceId);
    }


    public SimPigeon2(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
    }

    @Override
    public void setSimulatedYaw(double yawDegrees, double yawRateDegreesPerSecond) {
        Pigeon2SimState sim = getSimState();
        sim.setRawYaw(yawDegrees);
        sim.setAngularVelocityZ(yawRateDegreesPerSecond);
    }
}
