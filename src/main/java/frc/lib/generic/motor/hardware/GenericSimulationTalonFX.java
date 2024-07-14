package frc.lib.generic.motor.hardware;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;

public class GenericSimulationTalonFX extends Motor {
    private final TalonFX talonFX;

    private final StatusSignal<Double> positionSignal, velocitySignal, voltageSignal, currentSignal, temperatureSignal, closedLoopTarget;
    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private final TalonFXConfigurator talonConfigurator;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final MotionMagicVoltage positionMMRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityMMRequest = new MotionMagicVelocityVoltage(0);

    private MotorConfiguration currentConfiguration;

    private boolean shouldUseProfile = false;
    private int slotToUse = 0;

    public GenericSimulationTalonFX(String name, int deviceId) {
        super(name);

        talonFX = new TalonFX(deviceId);

        talonConfigurator = talonFX.getConfigurator();

        positionSignal = talonFX.getPosition().clone();
        velocitySignal = talonFX.getVelocity().clone();
        voltageSignal = talonFX.getMotorVoltage().clone();
        currentSignal = talonFX.getStatorCurrent().clone();
        temperatureSignal = talonFX.getDeviceTemp().clone();
        closedLoopTarget = talonFX.getClosedLoopReference().clone();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output) {
        switch (mode) {
            case PERCENTAGE_OUTPUT -> talonFX.setControl(dutyCycleRequest.withOutput(output));
            case VOLTAGE -> talonFX.setControl(voltageRequest.withOutput(output));

            case POSITION -> {
                if (shouldUseProfile) {
                    talonFX.setControl(positionMMRequest.withPosition(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(positionVoltageRequest.withPosition(output).withSlot(slotToUse));
                }
            }

            case VELOCITY -> {
                if (shouldUseProfile) {
                    talonFX.setControl(velocityMMRequest.withVelocity(output).withSlot(slotToUse));
                } else {
                    talonFX.setControl(velocityVoltageRequest.withVelocity(output).withSlot(slotToUse));
                }
            }

            case CURRENT ->
                    throw new UnsupportedOperationException("CTRE LOVES money and wants $150!!! dollars for this.. wtf.");
        }
    }

    @Override
    public double getVoltage() {
        return voltageSignal.refresh().getValue();
    }

    @Override
    public double getClosedLoopTarget() {
        return closedLoopTarget.refresh().getValue();
    }

    @Override
    public void stopMotor() {
        talonFX.stopMotor();
    }

    @Override
    public void setupSignalsUpdates(MotorSignal... signals) {
        for (MotorSignal signal : signals) {
            setupSignalUpdates(signal);
        }
    }

    public TalonFXSimState getSimulationState() {
        return talonFX.getSimState();
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        this.currentConfiguration = configuration;

        talonConfig.MotorOutput.Inverted = configuration.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.NeutralMode = configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        //Who the FUCK added this feature. CTRE should fucking fire him bruh
        talonConfig.Audio.BeepOnBoot = false;
        talonConfig.Audio.BeepOnConfig = false;

        talonConfig.Voltage.PeakForwardVoltage = 12;
        talonConfig.Voltage.PeakReverseVoltage = -12;

        talonConfig.Feedback.SensorToMechanismRatio = configuration.gearRatio;

        configureMotionMagic();

        setConfig0();
        setConfig1();
        setConfig2();

        applyCurrentLimits();

        talonConfig.ClosedLoopGeneral.ContinuousWrap = configuration.closedLoopContinuousWrap;

        slotToUse = configuration.slotToUse;

        talonFX.optimizeBusUtilization();

        return applyConfig();
    }

    private void configureMotionMagic() {
        if (currentConfiguration.profiledMaxVelocity == 0 || currentConfiguration.profiledTargetAcceleration == 0)
            return;

        talonConfig.MotionMagic.MotionMagicCruiseVelocity = currentConfiguration.profiledMaxVelocity;
        talonConfig.MotionMagic.MotionMagicAcceleration = currentConfiguration.profiledTargetAcceleration;

        shouldUseProfile = true;
    }

    private void setConfig0() {
        talonConfig.Slot0.kP = currentConfiguration.slot0.kP();
        talonConfig.Slot0.kI = currentConfiguration.slot0.kI();
        talonConfig.Slot0.kD = currentConfiguration.slot0.kD();

        talonConfig.Slot0.kA = currentConfiguration.slot0.kA();
        talonConfig.Slot0.kS = currentConfiguration.slot0.kS();
        talonConfig.Slot0.kV = currentConfiguration.slot0.kV();
        talonConfig.Slot0.kG = currentConfiguration.slot0.kG();

        if (currentConfiguration.slot0.gravityType() != null)
            talonConfig.Slot0.GravityType = currentConfiguration.slot0.gravityType();
    }

    private void setConfig1() {
        talonConfig.Slot1.kP = currentConfiguration.slot1.kP();
        talonConfig.Slot1.kI = currentConfiguration.slot1.kI();
        talonConfig.Slot1.kD = currentConfiguration.slot1.kD();
        talonConfig.Slot1.kA = currentConfiguration.slot1.kA();
        talonConfig.Slot1.kS = currentConfiguration.slot1.kS();
        talonConfig.Slot1.kV = currentConfiguration.slot1.kV();
        talonConfig.Slot1.kG = currentConfiguration.slot1.kG();

        if (currentConfiguration.slot1.gravityType() != null)
            talonConfig.Slot1.GravityType = currentConfiguration.slot1.gravityType();
    }

    private void setConfig2() {
        talonConfig.Slot2.kP = currentConfiguration.slot2.kP();
        talonConfig.Slot2.kI = currentConfiguration.slot2.kI();
        talonConfig.Slot2.kD = currentConfiguration.slot2.kD();
        talonConfig.Slot2.kA = currentConfiguration.slot2.kA();
        talonConfig.Slot2.kS = currentConfiguration.slot2.kS();
        talonConfig.Slot2.kV = currentConfiguration.slot2.kV();
        talonConfig.Slot2.kG = currentConfiguration.slot2.kG();

        if (currentConfiguration.slot2.gravityType() != null)
            talonConfig.Slot2.GravityType = currentConfiguration.slot2.gravityType();
    }

    private void applyCurrentLimits() {
        talonConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = currentConfiguration.dutyCycleOpenLoopRampPeriod;
        talonConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = currentConfiguration.dutyCycleClosedLoopRampPeriod;

        if (currentConfiguration.statorCurrentLimit != -1) {
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.StatorCurrentLimit = currentConfiguration.statorCurrentLimit;
        }

        if (currentConfiguration.supplyCurrentLimit != -1) {
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = currentConfiguration.supplyCurrentLimit;
        }
    }

    private boolean applyConfig() {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            statusCode = talonConfigurator.apply(talonConfig);
            counter--;
        }

        return statusCode == StatusCode.OK;
    }

    private void setupSignalUpdates(MotorSignal signal) {
        switch (signal.getType()) {
            case VELOCITY -> setupSignal(signal, velocitySignal);
            case POSITION -> setupSignal(signal, positionSignal);
            case VOLTAGE -> setupSignal(signal, voltageSignal);
            case CURRENT -> setupSignal(signal, currentSignal);
            case TEMPERATURE -> setupSignal(signal, temperatureSignal);
            case CLOSED_LOOP_TARGET -> setupSignal(signal, closedLoopTarget);
        }
    }

    private void setupSignal(final MotorSignal signal, final StatusSignal<Double> correspondingSignal) {
        correspondingSignal.setUpdateFrequency(signal.getUpdateRate());
    }
}