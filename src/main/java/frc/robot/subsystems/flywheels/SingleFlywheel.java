package frc.robot.subsystems.flywheels;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.mechanisms.SpeedMechanism2d;
import frc.lib.math.Conversions;

public class SingleFlywheel {
    private final Motor motor;
    private final SpeedMechanism2d speedMechanism2d;
    private final double flywheelDiameter;

    public SingleFlywheel(Motor motor, double flywheelDiameter) {
        this.motor = motor;
        this.flywheelDiameter = flywheelDiameter;

        speedMechanism2d =  new SpeedMechanism2d(motor.getName(), flywheelDiameter, 0.001, motor.getCurrentConfiguration().inverted);
    }

    public void periodic() {
        speedMechanism2d.updateMechanism(motor.getSystemVelocity(), motor.getClosedLoopTarget());
    }

    public boolean hasReachedTarget() {
        return motor.isAtSetpoint();
    }

    public void setTargetVelocity(double velocityRPS) {
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, velocityRPS);
    }

    public void setTargetTangentialVelocity(double velocityMPS) {
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, Conversions.mpsToRps(velocityMPS, flywheelDiameter));
    }

    public double getVoltage() {
        return motor.getVoltage();
    }

    public double getPosition() {
        return motor.getSystemPosition();
    }

    public double getVelocity() {
        return motor.getSystemVelocity();
    }

    public void setVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public void stop() {
        motor.stopMotor();
    }
}
