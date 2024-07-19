package frc.lib.generic.sensors.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.sensors.Sensor;
import frc.lib.generic.sensors.SensorInputs;
import frc.lib.generic.sensors.SensorSignal;

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
