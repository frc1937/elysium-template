package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.advantagekit.ChoosableLoggedInputs;
import org.littletonrobotics.junction.LogTable;

public class PigeonInputs implements ChoosableLoggedInputs {
    public static final int PIGEON_INPUTS_LENGTH = 7;

    public double gyroYawDegrees = 0;
    public double gyroRollDegrees = 0;
    public double gyroPitchDegrees = 0;

    public double[] timestamps = new double[0];

    public double[] threadGyroYawDegrees = new double[0];
    public double[] threadGyroPitchDegrees = new double[0];
    public double[] threadGyroRollDegrees = new double[0];

    private boolean[] signalsToLog;

    @Override
    public void setSignalsToLog(boolean[] signalsToLog) {
        this.signalsToLog = signalsToLog;
    }

    @Override
    public void toLog(LogTable table) {
        if (signalsToLog[0]) table.put("GyroYawDegrees", gyroYawDegrees);
        if (signalsToLog[1]) table.put("GyroRollDegrees", gyroRollDegrees);
        if (signalsToLog[2]) table.put("GyroPitchDegrees", gyroPitchDegrees);

        if (signalsToLog[3]) table.put("Timestamps", timestamps);

        if (signalsToLog[4]) table.put("ThreadGyroYawDegrees", threadGyroYawDegrees);
        if (signalsToLog[5]) table.put("ThreadGyroPitchDegrees", threadGyroPitchDegrees);
        if (signalsToLog[6]) table.put("ThreadGyroRollDegrees", threadGyroRollDegrees);
    }

    @Override
    public void fromLog(LogTable table) {
        gyroYawDegrees = table.get("GyroYawDegrees", gyroYawDegrees);
        gyroRollDegrees = table.get("GyroRollDegrees", gyroRollDegrees);
        gyroPitchDegrees = table.get("GyroPitchDegrees", gyroPitchDegrees);
        timestamps = table.get("Timestamps", timestamps);
        threadGyroYawDegrees = table.get("ThreadGyroYawDegrees", threadGyroYawDegrees);
        threadGyroPitchDegrees = table.get("ThreadGyroPitchDegrees", threadGyroPitchDegrees);
        threadGyroRollDegrees = table.get("ThreadGyroRollDegrees", threadGyroRollDegrees);
    }
}
