package frc.lib.generic.hardware.pigeon.hardware;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;
import frc.robot.poseestimation.poseestimator.OdometryThread;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

public class IMUPigeon extends Pigeon {
    private final WPI_PigeonIMU pigeon;

    private final boolean[] signalsToLog = new boolean[PigeonInputs.PIGEON_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();
    private final Queue<Double> timestampQueue = OdometryThread.getInstance().getTimestampQueue();

    public IMUPigeon(String name, int deviceNumber) {
        super(name);

        pigeon = new WPI_PigeonIMU(deviceNumber);
    }

    @Override
    public void resetConfigurations() {
        pigeon.configFactoryDefault();
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        pigeon.setYaw(yawDegrees);
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal) {
        signalsToLog[signal.getType().getId()] = true;

        if (!signal.useFasterThread()) return;

        signalsToLog[3] = true;
        signalsToLog[signal.getType().getId() + 4] = true;

        switch (signal.getType()) {
            case YAW -> signalQueueList.put("yaw", OdometryThread.getInstance().registerSignal(pigeon::getYaw));
            case ROLL -> signalQueueList.put("roll", OdometryThread.getInstance().registerSignal(pigeon::getRoll));
            case PITCH -> signalQueueList.put("pitch", OdometryThread.getInstance().registerSignal(pigeon::getPitch));
        }
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        if (pigeon == null) return;

        inputs.setSignalsToLog(signalsToLog);

        inputs.gyroYawDegrees = pigeon.getYaw();
        inputs.gyroRollDegrees = pigeon.getRoll();
        inputs.gyroPitchDegrees = pigeon.getPitch();

        if (signalQueueList.isEmpty()) return;

        inputs.timestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        if (signalQueueList.get("yaw") != null)
            inputs.threadGyroYawDegrees = signalQueueList.get("yaw").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("pitch") != null)
            inputs.threadGyroPitchDegrees = signalQueueList.get("pitch").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("roll") != null)
            inputs.threadGyroRollDegrees = signalQueueList.get("roll").stream().mapToDouble(Double::doubleValue).toArray();

        timestampQueue.clear();
        signalQueueList.forEach((k, v) -> v.clear());
    }
}
