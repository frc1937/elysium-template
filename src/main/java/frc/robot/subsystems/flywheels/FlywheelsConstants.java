package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

import static frc.lib.generic.hardware.motor.MotorSignal.SignalType.*;

public class FlywheelsConstants {
    private static final DCMotor FLYWHEEL_MOTOR_GEARBOX = DCMotor.getNeoVortex(1);

    private static final Motor LEFT_FLYWHEEL_MOTOR = MotorFactory.createSpark("Left Flywheel", 28, MotorProperties.SparkType.FLEX);
    private static final Motor RIGHT_FLYWHEEL_MOTOR = MotorFactory.createSpark("Right Flywheel", 15, MotorProperties.SparkType.FLEX);

    private static final MotorProperties.Slot
            LEFT_SLOT = new MotorProperties.Slot(0.0001, 0, 0, 0, 0, 0),
            RIGHT_SLOT = new MotorProperties.Slot(0.0001, 0, 0, 0, 0, 0),

            SIMULATION_SLOT = new MotorProperties.Slot(12, 0, 0, 0, 0, 0);

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

        motor.setupSignalsUpdates(
                new MotorSignal(CLOSED_LOOP_TARGET),
                new MotorSignal(VELOCITY),
                new MotorSignal(TEMPERATURE),
                new MotorSignal(VOLTAGE)
        );
    }

    protected static SingleFlywheel[] flywheels = {
            new SingleFlywheel(LEFT_FLYWHEEL_MOTOR, 6.0),
            new SingleFlywheel(RIGHT_FLYWHEEL_MOTOR, 6.0)
    };
}
