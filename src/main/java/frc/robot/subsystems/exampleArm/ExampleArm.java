package frc.robot.subsystems.exampleArm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;

import static frc.robot.subsystems.exampleArm.ExampleArmConstants.EXAMPLE_ARM_MOTOR;

public class ExampleArm extends GenericSubsystem {
    public Command setArmPosition(double position) {
        return new FunctionalCommand(
                () -> {},
                () -> EXAMPLE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> EXAMPLE_ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(EXAMPLE_ARM_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(EXAMPLE_ARM_MOTOR.getClosedLoopTarget());
    }

    public Command stopExampleArm() {
        return Commands.runOnce(EXAMPLE_ARM_MOTOR::stopMotor, this);
    }
}
