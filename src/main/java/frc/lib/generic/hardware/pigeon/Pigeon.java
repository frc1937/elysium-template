package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.hardware.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.Logger;

public class Pigeon implements LoggableHardware {
    private final PigeonInputs inputs = new PigeonInputs();
    private final String name;

    public Pigeon(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public void resetConfigurations() {}
    public double getYaw() {return inputs.gyroYawDegrees;}
    public double getPitch() {return inputs.gyroPitchDegrees;}
    public double getRoll() {return inputs.gyroRollDegrees;}
    public void setGyroYaw(double yawDegrees) {}

    /**
     * Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.
     */
    public void setupSignalUpdates(PigeonSignal signal) {}

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public PigeonInputs getInputs() {
        return inputs;
    }
    
    protected void refreshInputs(PigeonInputs inputs) {}
}
