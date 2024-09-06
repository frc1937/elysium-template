package frc.robot.subsystems.flywheels;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.math.Conversions;
import frc.lib.util.commands.ExecuteEndCommand;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.LEFT_FLYWHEEL_DIAMETER;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.RIGHT_FLYWHEEL_DIAMETER;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.flywheels;

public class Flywheels extends GenericSubsystem {
    private final int FLYWHEEL_INDEX_TO_LOG_SYSID = 0;

    public Command setTargetVelocity(double velocityRPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTargetVelocity(velocityRPS),
                this::stop,
                this
        );
    }

    public Command setTargetTangentialVelocity(double velocityMPS) {
        return new ExecuteEndCommand(
                () -> setFlywheelsTangentialVelocity(velocityMPS),
                this::stop,
                this
        );
    }

    @Override
    public void periodic() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.periodic();
        }
    }

    @Override
    public void sysIdDrive(double voltage) {
        flywheels[FLYWHEEL_INDEX_TO_LOG_SYSID].setVoltage(voltage);
    }

    @Override
    public void sysIdUpdateLog(SysIdRoutineLog log) {
        log.motor("Flywheel " + FLYWHEEL_INDEX_TO_LOG_SYSID)
                .voltage(Volts.of(flywheels[FLYWHEEL_INDEX_TO_LOG_SYSID].getVoltage()))
                .angularPosition(Rotations.of(flywheels[FLYWHEEL_INDEX_TO_LOG_SYSID].getPosition()))
                .angularVelocity(RotationsPerSecond.of(flywheels[FLYWHEEL_INDEX_TO_LOG_SYSID].getVelocity()));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                Volts.of(2),
                Second.of(9)
        );
    }

    public boolean hasReachedTarget() {
        for (SingleFlywheel flywheel : flywheels) {
            if (!flywheel.hasReachedTarget()) {
                return false;
            }
        }
        return true;
    }

    public void stop() {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.stop();
        }
    }

    public void setFlywheelsTangentialVelocity(double targetMPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetTangentialVelocity(targetMPS);
        }
    }

    public void setFlywheelsTargetVelocity(double targetRPS) {
        for (SingleFlywheel flywheel : flywheels) {
            flywheel.setTargetVelocity(targetRPS);
        }
    }

    public double getFlywheelTangentialVelocity() {
        return Math.min(
                Conversions.rpsToMps(flywheels[0].getVelocity(), LEFT_FLYWHEEL_DIAMETER),
                Conversions.rpsToMps(flywheels[1].getVelocity(), RIGHT_FLYWHEEL_DIAMETER)
        );
    }
}
