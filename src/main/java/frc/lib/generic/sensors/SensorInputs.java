package frc.lib.generic.sensors;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

public class SensorInputs implements ChoosableLoggedInputs {
    public int currentValue = 0;

    public double[] timestamp = new double[0];
    public int[] threadCurrentValue = new int[0];

    private boolean[] signalsToLog;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog == null) return;

        if (signalsToLog[0]) table.put("CurrentValue", currentValue);
        if (signalsToLog[1]) table.put("Timestamp", timestamp);
        if (signalsToLog[2]) table.put("ThreadCurrentValue", threadCurrentValue);
    }

    @Override
    public void fromLog(LogTable table) {
        currentValue = table.get("CurrentValue", currentValue);
        timestamp = table.get("Timestamp", timestamp);
        threadCurrentValue = table.get("ThreadCurrentValue", threadCurrentValue);
    }
}
