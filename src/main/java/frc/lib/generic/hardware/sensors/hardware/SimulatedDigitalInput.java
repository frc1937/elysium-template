package frc.lib.generic.hardware.sensors.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.hardware.sensors.SensorInputs;
import frc.lib.generic.hardware.sensors.SensorSignal;
import frc.lib.generic.hardware.sensors.Sensor;

public class SimulatedDigitalInput extends Sensor {
    private final boolean[] signalsToLog = new boolean[3];

    public SimulatedDigitalInput(String name) {
        super(name);
    }

    @Override
    public void setupSignalsUpdates(SensorSignal... signals) {
        for (SensorSignal signal : signals) {
            signalsToLog[signal.getType().getId()] = true;

            if (signal.useFasterThread()) {
                signalsToLog[1] = true;
                signalsToLog[signal.getType().getId() + 2] = true;
            }
        }
    }

    @Override
    public void refreshInputs(SensorInputs inputs) {
        inputs.setSignalsToLog(signalsToLog);

        inputs.currentValue = 1;

        inputs.threadCurrentValue = new int[]{inputs.currentValue};
        inputs.timestamp = new double[]{Timer.getFPGATimestamp()};
    }
}
