package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

public class FlywheelsConstants {
    private static final DCMotor FLYWHEEL_MOTOR_GEARBOX = DCMotor.getNeoVortex(1);

    private static final Motor LEFT_FLYWHEEL_MOTOR = MotorFactory.createSpark("Left Flywheel", 28, MotorProperties.SparkType.FLEX);
    private static final Motor RIGHT_FLYWHEEL_MOTOR = MotorFactory.createSpark("Right Flywheel", 15, MotorProperties.SparkType.FLEX);

    private static final MotorProperties.Slot
            LEFT_SLOT = new MotorProperties.Slot(0, 0, 0, 0.10904, 0.025022, 0.22468),
            RIGHT_SLOT = new MotorProperties.Slot(0, 0, 0, 0.10457, 0.037788, 0.05658),

    SIMULATION_SLOT = new MotorProperties.Slot(12, 0, 0, 0, 0, 0);

    protected static final double
            LEFT_FLYWHEEL_DIAMETER = 4,
            RIGHT_FLYWHEEL_DIAMETER = 3;

    static {
        configureMotor(LEFT_FLYWHEEL_MOTOR, true, LEFT_SLOT);
        configureMotor(RIGHT_FLYWHEEL_MOTOR, false, RIGHT_SLOT);
    }

    private static void configureMotor(Motor motor, boolean invert, MotorProperties.Slot slot) {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 40;
        configuration.statorCurrentLimit = 40;

        configuration.profiledTargetAcceleration = 15;
        configuration.profiledJerk = 10;

        configuration.slot0 = slot;

        configuration.closedLoopTolerance = 0.1;

        configuration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.FLYWHEEL,
                FLYWHEEL_MOTOR_GEARBOX,
                1,
                0.017
        );

        configuration.simulationSlot = SIMULATION_SLOT;

        motor.configure(configuration);

        motor.setupSignalUpdates(MotorSignal.VOLTAGE);
        motor.setupSignalUpdates(MotorSignal.TEMPERATURE);
        motor.setupSignalUpdates(MotorSignal.VELOCITY);
        motor.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }

    protected static SingleFlywheel[] flywheels = {
            new SingleFlywheel(LEFT_FLYWHEEL_MOTOR, 6.0),
            new SingleFlywheel(RIGHT_FLYWHEEL_MOTOR, 6.0)
    };
}
