package frc.lib.generic.motor;

import frc.lib.HardwareManager;
import frc.lib.generic.advantagekit.LoggableHardware;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Custom Motor class to allow switching and replacing motors quickly,
 * in addition of better uniformity across the code.
 */
public class Motor implements LoggableHardware {
    private final MotorInputs inputs = new MotorInputs();
    private final String name;

    private MotorConfiguration configuration;

    public Motor(String name) {
        this.name = name;

        periodic();
        HardwareManager.addHardware(this);
    }

    public String getName() {
        return name;
    }

    /**
     * Supplies an external position for the motor control system. This method allows
     * the feedforward and PID controllers to use an external encoder position value instead
     * of the system's position, allowing for more precise control using external {@link frc.lib.generic.encoder.Encoder Encoders}.
     *
     * @param position A {@link DoubleSupplier} providing the position to be used
     *                 by the motor control system.
     */
    public void setExternalPositionSupplier(DoubleSupplier position) { }

    /**
     * Supplies velocity from an external source for the motor control system. This method allows
     * the feedforward and PID controllers to use the externally supplied velocity value instead
     * of the system's calculated velocity, allowing for more precise control using external {@link frc.lib.generic.encoder.Encoder Encoders}.
     *
     * @param velocity A {@link DoubleSupplier} providing the velocity to be used
     *                 by the motor control system.
     */
    public void setExternalVelocitySupplier(DoubleSupplier velocity) { }

    /**
     * In case you need to re-set the slot on runtime, use this.
     *
     * @param slot       The new slot values
     * @param slotNumber The slot number to modify
     */
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        if (getCurrentConfiguration() == null) return;

        switch (slotNumber) {
            case 0 -> getCurrentConfiguration().slot0 = slot;
            case 1 -> getCurrentConfiguration().slot1 = slot;
            case 2 -> getCurrentConfiguration().slot2 = slot;
        }

        configure(getCurrentConfiguration());
    }

    /**
     * Sets the output of the motor based on the specified control mode and desired output value.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} - Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} - Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT} - Achieve a specific duty cycle percentage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} - Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} - Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profiledMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profiledTargetAcceleration profiledTargetAcceleration} are set.
     * The motor will calculate the needed feedforward based on the provided gains.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output) { }


    /**
     * Sets the output of the motor based on the specified control mode, desired output value, and custom feedforward.
     *
     * <p>This method utilizes the built-in feedforward and PID controller to achieve precise control
     * over the motor. The control mode determines how the output value is interpreted and applied
     * to the motor. The supported control modes include:
     * <ul>
     *   <li>{@link MotorProperties.ControlMode#CURRENT CURRENT} - Achieve a specific current.
     *   <li>{@link MotorProperties.ControlMode#VOLTAGE VOLTAGE} - Achieve a specific voltage.
     *   <li>{@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT} - Achieve a specific duty cycle percentage.
     *   <li>{@link MotorProperties.ControlMode#POSITION POSITION} - Achieve a specific position using advanced control.
     *   <li>{@link MotorProperties.ControlMode#VELOCITY VELOCITY} - Achieve a specific velocity using advanced control.
     * </ul>
     * </p>
     *
     * <p>For {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes,
     * a trapezoidal motion profile can optionally be used. To enable it, ensure both {@link MotorConfiguration#profiledMaxVelocity profiledMaxVelocity}
     * and {@link MotorConfiguration#profiledTargetAcceleration profiledTargetAcceleration} are set.
     * </p>
     *
     * <p>The custom feedforward is used to provide additional control over the motor output, allowing for fine-tuned
     * performance. Feedforward is applied only in {@link MotorProperties.ControlMode#POSITION POSITION} and {@link MotorProperties.ControlMode#VELOCITY VELOCITY} control modes.
     * </p>
     *
     * @param controlMode the control mode for the motor
     * @param output      the desired output value (amperes for {@link MotorProperties.ControlMode#CURRENT CURRENT}, volts for {@link MotorProperties.ControlMode#VOLTAGE VOLTAGE},
     *                    percentage for {@link MotorProperties.ControlMode#PERCENTAGE_OUTPUT PERCENTAGE_OUTPUT}, rotations for {@link MotorProperties.ControlMode#POSITION POSITION}
     *                    or rotations per second for {@link MotorProperties.ControlMode#VELOCITY VELOCITY})
     * @param feedforward the custom feedforward to be applied to the motor output
     */
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) { }

    /**
     * Set the idle mode of the motor
     *
     * @param idleMode The new idle mode
     */
    public void setIdleMode(MotorProperties.IdleMode idleMode) { }

    /**
     * Stop the motor
     */
    public void stopMotor() { }

    /**
     * Sets the encoder position of the motor to a specified value.
     *
     * <p>This method allows for manually setting the encoder position of the motor.
     * This can be useful for resetting the encoder position to a known reference point
     * or for calibrating the motor position in applications that require precise positional control.
     * </p>
     *
     * @param position the desired encoder position to set, in rotations.
     */
    public void setMotorEncoderPosition(double position) { }

    /**
     * Get the ID of the motor
     *
     * @return The ID of the motor
     */
    public int getDeviceID() { return -1; }

    /**
     * Retrieves the current position of the motor without any gearing applied.
     *
     * <p>This method returns the position of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current position of the motor in rotations
     */
    public double getMotorPosition() { return inputs.systemPosition / getCurrentConfiguration().gearRatio; }

    /**
     * Retrieves the current velocity of the motor, with no gearing applied.
     *
     * <p>This method returns the velocity of the motor as measured by the encoder,
     * without taking into account any gearing reductions or multipliers.
     * </p>
     *
     * @return the current velocity of the motor, in rotations per second (RPS).
     */
    public double getMotorVelocity() { return inputs.systemVelocity / getCurrentConfiguration().gearRatio; }

    /**
     * Get the current running through the motor (STATOR current)
     *
     * @Units In amps
     */
    public double getCurrent() { return inputs.current; }

    /**
     * Get the voltage running through the motor
     *
     * @Units In volts
     */
    public double getVoltage() { return inputs.voltage; }

    /**
     * Get the current target of the closed-loop PID
     */
    public double getClosedLoopTarget() { return inputs.target; }

    /**
     * Get the temperature of the motor
     *
     * @Units In celsius
     */
    public double getTemperature() { return inputs.temperature; }

    /**
     * Gearing applied
     *
     * @Units In rotations
     */
    public double getSystemPosition() { return inputs.systemPosition; }

    /**
     * Gearing applied
     *
     * @Units In rotations per second
     */
    public double getSystemVelocity() { return inputs.systemVelocity; }

    public void setFollowerOf(String name, int masterPort) { }

    /** Signals are lazily loaded - only these explicity called will be updated. Thus you must call this method. when using a signal.*/
    public void setupSignalsUpdates(MotorSignal... signals) { }

    public boolean configure(MotorConfiguration configuration) {
        this.configuration = configuration;
        return true;
    }

    /**
     * Gets the currently used configuration used by the motor. If this is not set, it will return null.
     *
     * @return The configuration
     */
    public MotorConfiguration getCurrentConfiguration() { return configuration; }

    /**
     * Gets the currently used configuration slot used by the motor. If this is not set, it will return null.
     *
     * @return The configuration slot
     */
    public MotorProperties.Slot getCurrentSlot() {
        return getSlot(getCurrentConfiguration().slotToUse, getCurrentConfiguration());
    }

    public MotorProperties.Slot getSlot(int slotToUse, MotorConfiguration currentConfiguration) {
        switch (slotToUse) {
            case 1 -> { return currentConfiguration.slot1; }
            case 2 -> { return currentConfiguration.slot2; }
            default -> { return currentConfiguration.slot0; }
        }
    }

    public boolean isAtSetpoint() {
        if (getCurrentConfiguration() == null || getCurrentConfiguration().closedLoopTolerance == 0)
            throw new UnsupportedOperationException("NUH UH");

        return Math.abs(getClosedLoopTarget() - getSystemPosition()) < getCurrentConfiguration().closedLoopTolerance;
    }

    protected void refreshInputs(MotorInputs inputs) { }

    @Override
    public void periodic() {
        refreshInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public MotorInputs getInputs() {
        return inputs;
    }
}
