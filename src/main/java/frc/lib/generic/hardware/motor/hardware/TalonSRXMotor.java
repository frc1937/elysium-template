package frc.lib.generic.hardware.motor.hardware;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.motor.*;

import java.util.function.DoubleSupplier;

public class TalonSRXMotor extends Motor {
    private final WPI_TalonSRX talonSRX;

    private MotorConfiguration currentConfiguration;

    public TalonSRXMotor(String name, int deviceNumber) {
        super(name);

        talonSRX = new WPI_TalonSRX(deviceNumber);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        switch (controlMode) {
            case POSITION -> talonSRX.set(ControlMode.Position, output);
            case VELOCITY -> talonSRX.set(ControlMode.Velocity, output);
            case PERCENTAGE_OUTPUT -> talonSRX.set(ControlMode.PercentOutput, output);
            case CURRENT -> talonSRX.set(ControlMode.Current, output);
            case VOLTAGE -> talonSRX.setVoltage(output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {
        throw new UnsupportedOperationException("I ain't implementing this lmfao");
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier position) {
        throw new UnsupportedOperationException("This will be the next pgishat movil project");
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocity) {
        throw new UnsupportedOperationException("This will be the next pgishat movil project");
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        talonSRX.config_kP(slotNumber, slot.kP());
        talonSRX.config_kI(slotNumber, slot.kI());
        talonSRX.config_kD(slotNumber, slot.kD());
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        talonSRX.setNeutralMode(idleMode == MotorProperties.IdleMode.COAST ? NeutralMode.Coast : NeutralMode.Brake);
    }

    @Override
    public void stopMotor() {
        talonSRX.stopMotor();
    }

    @Override
    public void setFollowerOf(String name, int masterPort) {
        talonSRX.set(ControlMode.Follower, masterPort);
    }

    @Override
    public double getClosedLoopTarget() {
        return talonSRX.getClosedLoopTarget();
    }

    @Override
    public double getTemperature() {
        return talonSRX.getTemperature();
    }

    @Override
    public double getMotorPosition() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getMotorVelocity() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getSystemPosition() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public double getSystemVelocity() {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        throw new UnsupportedOperationException("TalonSRX's don't have built-in relative encoders.");
    }

    @Override
    public int getDeviceID() {
        return talonSRX.getDeviceID();
    }

    @Override
    public void setupSignalsUpdates(MotorSignal... signal) {
        throw new UnsupportedOperationException("Talon SRX does NOT use signals. Use GenericTalonFX instead");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        talonSRX.configFactoryDefault();

        currentConfiguration = configuration;

        configureSlots(configuration);

        TalonSRXConfiguration talonSRXConfiguration = new TalonSRXConfiguration();

        talonSRXConfiguration.openloopRamp = configuration.dutyCycleOpenLoopRampPeriod;
        talonSRXConfiguration.closedloopRamp = configuration.dutyCycleClosedLoopRampPeriod;

        talonSRXConfiguration.feedbackNotContinuous = !configuration.closedLoopContinuousWrap;

        talonSRXConfiguration.peakCurrentLimit = (int) configuration.statorCurrentLimit;

        talonSRX.setInverted(configuration.inverted);

        return talonSRX.configAllSettings(talonSRXConfiguration) == ErrorCode.OK;
    }

    private void configureSlots(MotorConfiguration configuration) {
        talonSRX.config_kP(0, configuration.slot0.kP());
        talonSRX.config_kI(0, configuration.slot0.kI());
        talonSRX.config_kD(0, configuration.slot0.kD());

        talonSRX.config_kP(1, configuration.slot1.kP());
        talonSRX.config_kI(1, configuration.slot1.kI());
        talonSRX.config_kD(1, configuration.slot1.kD());

        talonSRX.config_kP(2, configuration.slot2.kP());
        talonSRX.config_kI(2, configuration.slot2.kI());
        talonSRX.config_kD(2, configuration.slot2.kD());
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (talonSRX == null) return;

        //todo: AKIT!!!

        inputs.current = talonSRX.getStatorCurrent();
        inputs.voltage = talonSRX.getMotorOutputVoltage();
        inputs.temperature = talonSRX.getTemperature();

        //todo: IMPLEMENT AKIT
    }
}
