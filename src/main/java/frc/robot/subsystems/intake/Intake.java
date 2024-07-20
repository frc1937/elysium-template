package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.intake.IntakeConstants.INTAKE_MOTOR;

public class Intake extends GenericSubsystem {
    public Command setIntakeSpeed(double speed) {
        return Commands.startEnd(
                () -> setPercentageOutput(speed),
                this::stop,
                this
        );
    }

    public Command stopIntake() {
        return Commands.runOnce(this::stop, this);
    }

    private void setPercentageOutput(double percentageOutput) {
        INTAKE_MOTOR.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, percentageOutput);
    }

    private void stop() {
        INTAKE_MOTOR.stopMotor();
    }
}
