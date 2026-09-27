package io.github.jamesehart.lightsim.motors;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import io.github.jamesehart.lightsim.LightSim;
import io.github.jamesehart.lightsim.SimMotor;

/**
 * A {@link SparkFlex} that LightSim simulates, including REV's onboard closed-loop control. On a real
 * robot it is exactly a SparkFlex. Defaults to a NEO Vortex (brushless) or CIM (brushed); pass a {@link
 * DCMotor} for anything else, e.g. {@code DCMotor.getNEO(1)}.
 */
public class SimSparkFlex extends SparkFlex implements SimMotor {
    private final DCMotor model;
    private final SparkFlexSim sim;

    public SimSparkFlex(int deviceId, MotorType type) {
        this(deviceId, type, type == MotorType.kBrushless ? DCMotor.getNeoVortex(1) : DCMotor.getCIM(1));
    }

    public SimSparkFlex(int deviceId, MotorType type, DCMotor motor) {
        super(deviceId, type);
        model = motor;
        if (RobotBase.isSimulation()) {
            sim = new SparkFlexSim(this, motor);
            LightSim.register(this);
        } else {
            sim = null;
        }
    }

    @Override
    public DCMotor getMotorModel() {
        return model;
    }

    @Override
    public double getAppliedVolts(double supplyVolts) {
        return sim.getAppliedOutput() * supplyVolts;
    }

    @Override
    public void setState(double rotorPositionRot, double rotorVelocityRps, double mechanismVelocityRps,
            double statorCurrentAmps, double supplyVolts, double dtSeconds) {
        boolean absolute = configAccessor.closedLoop.getFeedbackSensor() == FeedbackSensor.kAbsoluteEncoder;
        double rotorRpm = rotorVelocityRps * 60;
        double encoderVelocity = rotorRpm * configAccessor.encoder.getVelocityConversionFactor();
        if (absolute) {
            // An absolute encoder sits on the mechanism, after the gearing.
            double absoluteVelocity =
                    mechanismVelocityRps * 60 * configAccessor.absoluteEncoder.getVelocityConversionFactor();
            sim.iterate(absoluteVelocity, supplyVolts, dtSeconds);
            sim.getRelativeEncoderSim().iterate(encoderVelocity, dtSeconds);
        } else {
            sim.iterate(encoderVelocity, supplyVolts, dtSeconds);
        }
        sim.setMotorCurrent(Math.abs(statorCurrentAmps));
    }
}
