package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonFactory;
import frc.lib.generic.hardware.pigeon.PigeonSignal;
import frc.lib.math.AdvancedSwerveKinematics;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

public class SwerveConstants {
    public static final double DRIVE_GEAR_RATIO = (6.75);
    public static final double STEER_GEAR_RATIO = (150.0 / 7.0);

    public static final double WHEEL_DIAMETER = Meters.convertFrom(4, Inch);

    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final double MAX_SPEED_MPS = 5.1;
    public static final double MAX_ROTATION_RAD_PER_S = 3 * Math.PI;

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();
    
    private static final Translation2d[] moduleLocations = {
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    };

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(moduleLocations);
    public static final AdvancedSwerveKinematics ADVANCED_KINEMATICS = new AdvancedSwerveKinematics(moduleLocations);

    public static final double
            DRIVE_NEUTRAL_DEADBAND = 0.2,
            ROTATION_NEUTRAL_DEADBAND = 0.2;

    /**
     * Units of RADIANS for everything.
     */
    static final ProfiledPIDController ROTATION_CONTROLLER = new ProfiledPIDController(
            1, 0, 0.0015,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2)
    );

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0, 0),
            new PIDConstants(2, 0, 0),
            MAX_SPEED_MPS,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, false)
    );

    static final Pigeon GYRO = PigeonFactory.createIMU("GYRO", 30);

    static {
        configureGyro();
        configureRotationController();
    }

    private static void configureGyro() {
        GYRO.resetConfigurations();
        GYRO.setupSignalUpdates(new PigeonSignal(PigeonSignal.SignalType.YAW, true));
    }

    private static void configureRotationController() {
        ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        ROTATION_CONTROLLER.setTolerance(Units.degreesToRadians(0.5));
    }
}
