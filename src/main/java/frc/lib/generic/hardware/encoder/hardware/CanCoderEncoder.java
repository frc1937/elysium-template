package frc.lib.generic.hardware.encoder.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.lib.generic.hardware.encoder.*;
import frc.robot.poseestimation.poseestimator.OdometryThread;

import java.util.*;

/**
 * Wrapper class for the CAN encoder.
 * Verify its setup is correct via this:
 * <a href="https://store.ctr-electronics.com/content/user-manual/CANCoder%20User">CTRE CANcoder PDF</a>'s%20Guide.pdf
 */
public class CanCoderEncoder extends Encoder {
    private final CANcoder canCoder;
    private final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private final boolean[] signalsToLog = new boolean[4];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private final List<StatusSignal<Double>> signalsToUpdateList = new ArrayList<>();
    private final StatusSignal<Double> positionSignal, velocitySignal;

    public CanCoderEncoder(String name, int canCoderID) {
        super(name);

        canCoder = new CANcoder(canCoderID);

        positionSignal = canCoder.getPosition().clone();
        velocitySignal = canCoder.getVelocity().clone();
    }

    @Override
    public void setSignalUpdateFrequency(EncoderSignal signal) {
        signalsToLog[signal.getType().getId()] = true;

        switch (signal.getType()) {
            case POSITION -> setupSignal(signal, positionSignal);
            case VELOCITY -> setupSignal(signal, velocitySignal);
        }

        if (!signal.useFasterThread()) return;

        signalsToLog[signal.getType().getId() + 2] = true;

        switch (signal.getType()) {
            case POSITION -> signalQueueList.put("position", OdometryThread.getInstance().registerSignal(this::getEncoderPositionPrivate));
            case VELOCITY -> signalQueueList.put("velocity", OdometryThread.getInstance().registerSignal(this::getEncoderVelocityPrivate));
        }
    }

    @Override
    public boolean configure(EncoderConfiguration encoderConfiguration) {
        canCoderConfig.MagnetSensor.MagnetOffset = encoderConfiguration.offsetRotations;

        canCoderConfig.MagnetSensor.SensorDirection = encoderConfiguration.invert ?
                SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfig.MagnetSensor.AbsoluteSensorRange = encoderConfiguration.sensorRange == EncoderProperties.SensorRange.ZeroToOne
                ? AbsoluteSensorRangeValue.Unsigned_0To1 : AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        canCoder.optimizeBusUtilization();

        return applyConfig();
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = canCoder.getConfigurator().apply(canCoderConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }

    @Override
    protected void refreshInputs(EncoderInputs inputs) {
        if (canCoder == null) return;

        inputs.setSignalsToLog(signalsToLog);

        BaseStatusSignal.refreshAll(signalsToUpdateList.toArray(new BaseStatusSignal[0]));

        inputs.position = getEncoderPositionPrivate();
        inputs.velocity = getEncoderVelocityPrivate();

        if (signalQueueList.isEmpty()) return;

        if (signalQueueList.get("position") != null)
            inputs.threadPosition = signalQueueList.get("position").stream().mapToDouble(Double::doubleValue).toArray();
        if (signalQueueList.get("velocity") != null)
            inputs.threadVelocity = signalQueueList.get("velocity").stream().mapToDouble(Double::doubleValue).toArray();

        signalQueueList.forEach((k, v) -> v.clear());
    }

    private double getEncoderPositionPrivate() {
        return positionSignal.getValue();
    }

    private double getEncoderVelocityPrivate() {
        return velocitySignal.getValue();
    }

    private void setupSignal(final EncoderSignal signal, final StatusSignal<Double> correspondingSignal) {
        signalsToUpdateList.add(correspondingSignal);
        correspondingSignal.setUpdateFrequency(signal.getUpdateRate());
    }
}
