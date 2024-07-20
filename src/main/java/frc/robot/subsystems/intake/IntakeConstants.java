package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorFactory;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.simulation.SimulationProperties;

public class IntakeConstants {
    protected static final Motor INTAKE_MOTOR = MotorFactory.createTalonSRX("Intake", 5);

    static {
        configureMotor();
    }

    private static void configureMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = true;

        configuration.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1.0,
                0.003
                );

        INTAKE_MOTOR.configure(configuration);
    }
}
