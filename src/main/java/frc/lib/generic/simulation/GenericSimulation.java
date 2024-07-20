package frc.lib.generic.simulation;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.generic.hardware.motor.hardware.SimulationTalonFX;

import java.util.ArrayList;
import java.util.List;

public abstract class GenericSimulation {
    /**
     * This instance is shared between all inheritors
     */
    private static final List<GenericSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private final SimulationTalonFX motor;
    private final TalonFXSimState motorSimulatedState;

    protected GenericSimulation() {
        REGISTERED_SIMULATIONS.add(this);

        motor = new SimulationTalonFX("Amit Sucher", REGISTERED_SIMULATIONS.size() - 1);

        MotorSignal[] signals = List.of(new MotorSignal(MotorSignal.SignalType.CURRENT),
                new MotorSignal(MotorSignal.SignalType.POSITION),
                new MotorSignal(MotorSignal.SignalType.VELOCITY),
                new MotorSignal(MotorSignal.SignalType.VOLTAGE),
                new MotorSignal(MotorSignal.SignalType.TEMPERATURE),
                new MotorSignal(MotorSignal.SignalType.CLOSED_LOOP_TARGET)).toArray(new MotorSignal[0]);

        motor.setupSignalsUpdates(signals);

        motorSimulatedState = motor.getSimulationState();
        motorSimulatedState.setSupplyVoltage(12); //Voltage compensation.
    }

    /**
     * This method should be called on the @simulationPeriodic
     */
    public static void updateAllSimulations() {
        for (GenericSimulation simulation : REGISTERED_SIMULATIONS) {
            simulation.updateSimulation();
        }
    }

    public int getDeviceID() {
        return motor.getDeviceID();
    }

    public void configure(MotorConfiguration configuration) {
        motor.configure(configuration);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        motor.setOutput(controlMode, output);
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    public double getTarget() {
        return motor.getClosedLoopTarget();
    }

    private void updateSimulation() {
        setVoltage(motorSimulatedState.getMotorVoltage());
        update();

        motorSimulatedState.setRawRotorPosition(getPositionRotations());
        motorSimulatedState.setRotorVelocity(getVelocityRotationsPerSecond());
    }

    public abstract double getPositionRotations();

    public abstract double getVelocityRotationsPerSecond();

    public abstract double getCurrent();

    //These have weaker access because they're used in this package only.
    abstract void update();

    abstract void setVoltage(double voltage);
}
