package frc.lib.generic.motor.hardware;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorInputsAutoLogged;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.simulation.GenericSimulation;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class SimulatedMotor extends Motor {
    private MotorConfiguration currentConfiguration;

    private GenericSimulation simulation;

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
        if (simulation != null ) {
            simulation.stop();
        }
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        switch (slotNumber) {
            case 0 -> currentConfiguration.slot0 = slot;
            case 1 -> currentConfiguration.slot1 = slot;
            case 2 -> currentConfiguration.slot2 = slot;
        }

        configure(currentConfiguration);
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
    protected void refreshInputs(MotorInputsAutoLogged inputs) {
        if (CURRENT_MODE != GlobalConstants.Mode.SIMULATION) {
            throw new RuntimeException("This motor should NEVER be initialized manually! Use the factory methods instead!");
        }

        if (simulation == null) return;

        inputs.systemPosition = simulation.getPositionRotations();
        inputs.systemVelocity = simulation.getVelocityRotationsPerSecond();
        inputs.voltage = simulation.getVoltage();
        inputs.current = simulation.getCurrent();

        inputs.temperature = 0.0;
        inputs.target = simulation.getTarget();

        inputs.threadSystemPosition = new double[]{inputs.systemPosition};
        inputs.threadSystemVelocity = new double[]{inputs.systemVelocity};
        inputs.threadVoltage = new double[]{inputs.voltage};
        inputs.threadCurrent = new double[]{inputs.current};
        inputs.threadTemperature = new double[]{inputs.temperature};
        inputs.threadTarget = new double[]{inputs.target};

        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};
    }
}