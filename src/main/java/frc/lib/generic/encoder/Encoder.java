package frc.lib.generic.encoder;

import frc.lib.generic.advantagekit.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Encoder implements LoggableHardware {
    private final EncoderInputs inputs = new EncoderInputs();
    private final String name;

    public Encoder(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderPositionSource(DoubleSupplier positionSource) {}
    /** This is required for sim to function correctly. In real, this won't do anything. */
    public void setSimulatedEncoderVelocitySource(DoubleSupplier velocitySource) {}

    /** Returns the encoder position, in Rotations*/
    public double getEncoderPosition() {return inputs.position; }
    public double getEncoderVelocity() {return inputs.velocity; }

    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    public void setSignalUpdateFrequency(EncoderSignal signal) {}

    public boolean configure(EncoderConfiguration encoderConfiguration) { return true; }

    protected void refreshInputs(EncoderInputs inputs) { }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public EncoderInputs getInputs() {
        return inputs;
    }
}
