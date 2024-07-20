package frc.lib.generic.hardware.pigeon.hardware;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.hardware.pigeon.Pigeon;
import frc.lib.generic.hardware.pigeon.PigeonInputs;
import frc.lib.generic.hardware.pigeon.PigeonSignal;

import static frc.robot.GlobalConstants.ROBOT_PERIODIC_LOOP_TIME;
import static frc.robot.RobotContainer.SWERVE;

public class SimulatedIMU extends Pigeon {
    private double simulatedYawDegrees = 0;
    private final boolean[] signalsToLog = new boolean[PigeonInputs.PIGEON_INPUTS_LENGTH];

    public SimulatedIMU(String name) {
        super(name);
    }

    public void update(double omegaRadiansPerSecond, double timeSeconds) {
        simulatedYawDegrees += Units.radiansToDegrees(omegaRadiansPerSecond * timeSeconds);
    }

    @Override
    public double getYaw() {
        return simulatedYawDegrees;
    }

    @Override
    public void setGyroYaw(double yawDegrees) {
        simulatedYawDegrees = yawDegrees;
    }

    @Override
    public void setupSignalUpdates(PigeonSignal signal) {
        if (signal.useFasterThread()) {
            signalsToLog[3] = true;
            signalsToLog[signal.getType().getId() + 4] = true;
        }

        signalsToLog[signal.getType().getId()] = true;
    }

    @Override
    protected void refreshInputs(PigeonInputs inputs) {
        if (SWERVE == null) return; //The gyro is initialized before the swerve at the beginning
        inputs.setSignalsToLog(signalsToLog);

        update(SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, ROBOT_PERIODIC_LOOP_TIME);

        inputs.gyroYawDegrees = getYaw();

        inputs.threadGyroYawDegrees = new double[]{inputs.gyroYawDegrees};
        inputs.timestamps = new double[]{Timer.getFPGATimestamp()};
    }
}
