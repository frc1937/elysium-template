package frc.lib.generic.encoder.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderInputs;
import frc.lib.generic.encoder.EncoderSignal;

import java.util.function.DoubleSupplier;

public class SimulatedCanCoder extends Encoder {
    private DoubleSupplier positionSupplier = () -> 0;
    private DoubleSupplier velocitySupplier = () -> 0;

    private final boolean[] signalsToLog = new boolean[5];

    public SimulatedCanCoder(String name) {
        super(name);
    }

    @Override
    public void setSignalUpdateFrequency(EncoderSignal signal) {
        signalsToLog[signal.getType().getId()] = true;

        if (signal.useFasterThread()) {
            signalsToLog[signal.getType().getId() + 3] = true;
            signalsToLog[2] = true;
        }
    }

    @Override
    public void setSimulatedEncoderPositionSource(DoubleSupplier supplier) {
        positionSupplier = supplier;
    }

    @Override
    public void setSimulatedEncoderVelocitySource(DoubleSupplier supplier) {
        velocitySupplier = supplier;
    }

    @Override
    protected void refreshInputs(EncoderInputs inputs) {
        inputs.setSignalsToLog(signalsToLog);

        if (positionSupplier == null || velocitySupplier == null) {
            return;
        }

        inputs.position = positionSupplier.getAsDouble();
        inputs.velocity = velocitySupplier.getAsDouble();

        inputs.threadPosition = new double[]{inputs.position};
        inputs.threadVelocity = new double[]{inputs.velocity};
        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};
    }
}
