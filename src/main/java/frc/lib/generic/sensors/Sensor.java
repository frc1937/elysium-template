package frc.lib.generic.sensors;

import frc.lib.generic.advantagekit.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.Logger;

public class Sensor implements LoggableHardware {
    private final SensorInputs inputs = new SensorInputs();
    private final String name;

    public Sensor(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public void setupSignalsUpdates(SensorSignal... signals) { }

    public int get() { return inputs.currentValue; }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public void refreshInputs(SensorInputs inputs) { }

    public SensorInputs getInputs() { return inputs; }
}
