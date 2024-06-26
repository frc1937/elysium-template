package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Optimizations;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIO {
    private boolean openLoop = true;

    private final SwerveModuleInputsAutoLogged swerveModuleInputs = new SwerveModuleInputsAutoLogged();
    private final String name;

    private SwerveModuleState targetState  = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    public SwerveModuleIO(String name) {
        this.name = name;
    }

    public String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    public void periodic() {
        refreshInputs(swerveModuleInputs);
        Logger.processInputs(getLoggingPath(), swerveModuleInputs);

        modulePeriodic();
    }

    void setTargetState(SwerveModuleState state) {
        this.targetState = SwerveModuleState.optimize(state, getCurrentAngle());

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
    SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                swerveModuleInputs.odometryUpdatesDriveDistanceMeters[odometryUpdateIndex],
                Rotation2d.fromDegrees(swerveModuleInputs.odometryUpdatesSteerAngleDegrees[odometryUpdateIndex])
        );
    }

    protected Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(swerveModuleInputs.steerAngleDegrees);
    }

    protected SwerveModuleState getTargetState() {
        return targetState;
    }

    protected void setOpenLoop(boolean shouldBeOpenLoop) {
        openLoop = shouldBeOpenLoop;
    }

    protected void setTargetAngle(Rotation2d angle) {}
    protected void setTargetVelocity(double velocityMetresPerSecond, boolean openLoop) {}
    protected void modulePeriodic() {}

    protected void stop() {}

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(swerveModuleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    protected void refreshInputs(SwerveModuleInputsAutoLogged swerveModuleInputs) {}

    @AutoLog
    public static class SwerveModuleInputs {
        public double steerAngleDegrees = 0;
        public double steerVoltage = 0;

        public double driveVelocityMetersPerSecond = 0;
        public double driveDistanceMeters = 0;
        public double driveVoltage = 0;

        public double[] odometryUpdatesDriveDistanceMeters = new double[0];
        public double[] odometryUpdatesSteerAngleDegrees = new double[0];
    }
}
