package frc.robot.subsystems.flywheels;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.mechanisms.SpeedMechanism2d;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

public class SingleFlywheel {
    private final Motor motor;
    private final SpeedMechanism2d speedMechanism2d;
    private final double flywheelDiameter;

    public SingleFlywheel(Motor motor, double flywheelDiameter) {
        this.motor = motor;
        this.flywheelDiameter = flywheelDiameter;

        speedMechanism2d =  new SpeedMechanism2d(motor.getName(), flywheelDiameter, 0.001, motor.getCurrentConfiguration().inverted);
        Logger.recordOutput("LeConsole" + motor.getName(), "SingleFlywheel: " + motor.getName() + " created, inverted: " + motor.getCurrentConfiguration().inverted);
    }

    public void periodic() {
        speedMechanism2d.updateMechanism(motor.getSystemVelocity());
    }

    public boolean hasReachedTarget() {
        return motor.isAtSetpoint();
    }

    public void setTargetVelocity(double velocityRPS) {
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, velocityRPS);
        speedMechanism2d.setTargetVelocity(velocityRPS);
    }

    public void setTargetTangentialVelocity(double velocityMPS) {
        motor.setOutput(MotorProperties.ControlMode.VELOCITY, Conversions.mpsToRps(velocityMPS, flywheelDiameter));
        speedMechanism2d.setTargetVelocity(Conversions.mpsToRps(velocityMPS, flywheelDiameter));
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

    public double getTargetVelocityRPS() {
        return motor.getClosedLoopTarget();
    }

    public void setVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public void stop() {
        motor.stopMotor();
    }
}
