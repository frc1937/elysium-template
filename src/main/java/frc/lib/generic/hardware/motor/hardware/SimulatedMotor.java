package frc.lib.generic.hardware.motor.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.motor.*;
import frc.lib.generic.simulation.GenericSimulation;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SimulatedMotor extends Motor {
    private MotorConfiguration currentConfiguration;
    private GenericSimulation simulation;

    private final boolean[] signalsToLog = new boolean[]{
            false, false, false, false, false, false, false, false, false, false, false, false, false
    };

    public SimulatedMotor(String name) {
        super(name);

        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION) {
            throw new RuntimeException("DO NOT Initialize THIS MOTOR! Use the factory methods instead!");
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        if (simulation != null) {
            simulation.setOutput(controlMode, output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        if (simulation != null) {
            simulation.setOutput(mode, output);
        }
    }

    @Override
    public void stopMotor() {
        if (simulation != null) {
            simulation.stop();
        }
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        simulation = configuration.simulationProperties.getSimulationFromType();

        configuration.slot0 = configuration.simulationSlot;
        configuration.slot1 = configuration.simulationSlot;
        configuration.slot2 = configuration.simulationSlot;

        simulation.configure(configuration);

        return true;
    }

    @Override
    public int getDeviceID() {
        return simulation.getDeviceID();
    }

    @Override
    public void setupSignalsUpdates(MotorSignal... signals) {
        for (MotorSignal signal : signals) {
            if (signal.useFasterThread()) signalsToLog[6] = true;

            signalsToLog[signal.getType().getId()] = true;
            signalsToLog[signal.getType().getId() + 7] = true;
        }
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        inputs.setSignalsToLog(signalsToLog);

        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION) {
            throw new RuntimeException("This motor should NEVER be initialized manually! Use the factory methods instead!");
        }

        if (simulation == null) return;

        inputs.voltage = simulation.getVoltage();
        inputs.current = simulation.getCurrent();
        inputs.temperature = 0;
        inputs.target = simulation.getTarget();
        inputs.systemPosition = simulation.getPositionRotations();
        inputs.systemVelocity = simulation.getVelocityRotationsPerSecond();

        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};

        inputs.threadVoltage = new double[]{inputs.voltage};
        inputs.threadCurrent = new double[]{inputs.current};
        inputs.threadTemperature = new double[]{inputs.temperature};
        inputs.threadTarget = new double[]{inputs.target};
        inputs.threadSystemPosition = new double[]{inputs.systemPosition};
        inputs.threadSystemVelocity = new double[]{inputs.systemVelocity};
    }
}
