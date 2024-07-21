package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.MotorInputs;

import java.util.Map;
import java.util.Queue;

public class MotorUtilities {
    static void handleThreadedInputs(MotorInputs inputs, Map<String, Queue<Double>> signalQueueList) {
        if (signalQueueList.isEmpty()) return;

        if (signalQueueList.get("position") != null)
            inputs.threadSystemPosition = signalQueueList.get("position").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("velocity") != null)
            inputs.threadSystemVelocity = signalQueueList.get("velocity").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("voltage") != null)
            inputs.threadVoltage = signalQueueList.get("voltage").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("current") != null)
            inputs.threadCurrent = signalQueueList.get("current").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("temperature") != null)
            inputs.threadTemperature = signalQueueList.get("temperature").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("target") != null)
            inputs.threadTarget = signalQueueList.get("target").stream().mapToDouble(Double::doubleValue).toArray();

        signalQueueList.forEach((k, v) -> v.clear());
    }
}
