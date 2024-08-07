package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.EncoderSignal;
import frc.lib.generic.encoder.hardware.GenericCanCoder;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.generic.motor.MotorSignal;
import frc.lib.generic.motor.hardware.GenericSpark;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.simulation.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.*;
import static frc.lib.generic.motor.MotorSignal.SignalType.*;

public class ArmConstants {
    static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(0.5),
            Volts.of(2),
            Seconds.of(10)
    );

    static final double TOLERANCE_ROTATIONS = Units.degreesToRotations(0.5);

    static final SingleJointedArmMechanism2d ARM_MECHANISM =
            new SingleJointedArmMechanism2d("ArmMechanism", new Color8Bit(Color.kRed));

    static final Motor ARM_MOTOR = new GenericSpark("ARM_MOTOR", 1, MotorProperties.SparkType.FLEX);
    static final Encoder ABSOLUTE_ARM_ENCODER = new GenericCanCoder("ARM_ENCODER", 22);

    static final double PITCH_GEAR_RATIO = 149;

    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);

    static {
        configureMotor();
        configureEncoder();
    }

    private static void configureEncoder() {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = true;
        encoderConfiguration.offsetRotations = PIVOT_ENCODER_OFFSET.getRotations();
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.NegativeHalfToHalf;

        ABSOLUTE_ARM_ENCODER.configure(encoderConfiguration);

        ABSOLUTE_ARM_ENCODER.setSignalUpdateFrequency(new EncoderSignal(EncoderSignal.SignalType.POSITION));
        ABSOLUTE_ARM_ENCODER.setSignalUpdateFrequency(new EncoderSignal(EncoderSignal.SignalType.VELOCITY));
    }

    private static void configureMotor() {
        MotorConfiguration motorConfiguration = new MotorConfiguration();

        motorConfiguration.gearRatio = PITCH_GEAR_RATIO;

        motorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;
        motorConfiguration.supplyCurrentLimit = 40;

        motorConfiguration.profiledTargetAcceleration = 0.35;
        motorConfiguration.profiledMaxVelocity = 0.5;

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.slot0 = new MotorProperties.Slot(
                0, 0, 0,
                15.625, 0.85843, 0.097736, 0.21308,
                GravityTypeValue.Arm_Cosine
        );

        motorConfiguration.closedLoopTolerance = TOLERANCE_ROTATIONS;

        motorConfiguration.simSlot = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getKrakenX60(1),
                150.0,
                0.2,
                0.03,
                Rotation2d.fromDegrees(-20),
                Rotation2d.fromDegrees(120),
                true
        );

        ARM_MOTOR.configure(motorConfiguration);

        ARM_MOTOR.setupSignalsUpdates(new MotorSignal(POSITION));
        ARM_MOTOR.setupSignalsUpdates(new MotorSignal(VELOCITY));
        ARM_MOTOR.setupSignalsUpdates(new MotorSignal(VOLTAGE));

        ARM_MOTOR.setExternalPositionSupplier(ABSOLUTE_ARM_ENCODER::getEncoderPosition);
        ARM_MOTOR.setExternalVelocitySupplier(ABSOLUTE_ARM_ENCODER::getEncoderVelocity);
    }
}
