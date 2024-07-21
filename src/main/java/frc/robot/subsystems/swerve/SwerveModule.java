package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderInputs;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.lib.math.Optimizations;

import java.util.Arrays;

import static frc.lib.math.Conversions.rotationsToMetres;
import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_DIAMETER;

public class SwerveModule {
    private final Motor steerMotor, driveMotor;
    private final Encoder steerEncoder;

    private SwerveModuleState targetState = new SwerveModuleState();
    private boolean openLoop = true;

    public SwerveModule(Motor driveMotor, Motor steerMotor, Encoder steerEncoder) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.steerEncoder = steerEncoder;
    }

    protected void setTargetState(SwerveModuleState state) {
        this.targetState = Optimizations.optimize(state, getCurrentAngle());

        final double optimizedVelocity = Optimizations.reduceSkew(targetState.speedMetersPerSecond, targetState.angle, getCurrentAngle());

        setTargetAngle(targetState.angle);
        setTargetVelocity(optimizedVelocity, openLoop);
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    protected SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                getDriveMetersTraveled(getDriveMotorInputs().threadSystemPosition)[odometryUpdateIndex],
                Rotation2d.fromRotations(getSteerEncoderInputs().threadPosition[odometryUpdateIndex])
        );
    }

    protected void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    protected void setTargetVelocity(double velocityMetresPerSecond, boolean openLoop) {
        if (openLoop) {
            final double targetPowerOpenLoop = VOLTAGE_COMPENSATION_SATURATION * (velocityMetresPerSecond / MAX_SPEED_MPS);
            driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, targetPowerOpenLoop);
        } else {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        }
    }

    protected void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(driveMotor.getSystemVelocity(), getCurrentAngle());
    }

    protected SwerveModuleState getTargetState() {
        return targetState;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerEncoder.getEncoderPosition());
    }

    private EncoderInputs getSteerEncoderInputs() {
        return steerEncoder.getInputs();
    }

    private MotorInputs getDriveMotorInputs() {
        return driveMotor.getInputs();
    }

    private double[] getDriveMetersTraveled(double[] rotationsPositions) {
        return Arrays.stream(rotationsPositions).map(position -> rotationsToMetres(position, WHEEL_DIAMETER)).toArray();
    }
}
