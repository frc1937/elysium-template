package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.real.RealSwerveConstants;
import frc.robot.subsystems.swerve.simulation.SimulatedSwerveConstants;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

public abstract class SwerveConstants {
    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    static final LoggedTunableNumber ROTATION_KP = new LoggedTunableNumber("Swerve/RotationKP", 1);

    static final LoggedTunableNumber ROTATION_MAX_VELOCITY = new LoggedTunableNumber("Swerve/RotationMaxVelocity", Math.PI),
            ROTATION_MAX_ACCELERATION = new LoggedTunableNumber("Swerve/RotationMaxAcceleration", Math.PI );

    /** Units of RADIANS for everything. */
    protected static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            ROTATION_KP.get(), 0, 0.015, new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY.get(), ROTATION_MAX_ACCELERATION.get())
    );

    public static final double WHEEL_DIAMETER = Meters.convertFrom(4, Inch);

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();

    static final double MAX_SPEED_MPS = 5.1;
    static final double MAX_ROTATION_RAD_PER_S = 3 * Math.PI;


    static SwerveConstants generateConstants() {
        ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        ROTATION_CONTROLLER.setTolerance(Units.degreesToRadians(0.5));

        if (GlobalConstants.CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedSwerveConstants();
        }

        return new RealSwerveConstants();
    }


    protected abstract SwerveModuleIO[] getSwerveModules();
}
