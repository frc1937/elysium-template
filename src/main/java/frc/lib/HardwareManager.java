package frc.lib;

import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.generic.advantagekit.LoggableHardware;
import frc.lib.generic.simulation.GenericSimulation;
import frc.robot.GlobalConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static frc.robot.GlobalConstants.CURRENT_MODE;
import static frc.robot.GlobalConstants.FASTER_THREAD_LOCK;

//Credit to team 418 for this
public enum HardwareManager {
    INSTANCE;

    HardwareManager() {}

    private static final List<LoggableHardware> hardware = new ArrayList<>();
    private static final List<Runnable> periodicRunnable = new ArrayList<>();

    /**
     * Initialize and start logging
     * <p>
     * Call this at the beginning of <code>robotInit()</code>.
     * <p>
     * To enable replay, set the variable <code>CURRENT_MODE=REPLAY</code>
     *
     * @param robot Robot object
     */
    public static void initialize(LoggedRobot robot) {
        boolean shouldWriteLogs = true; //for when the roborio doesn't have enough space...

        String logPath = Filesystem.getDeployDirectory().getPath() + "/logs/";

        if (CURRENT_MODE == GlobalConstants.Mode.REAL || CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            Logger.addDataReceiver(new NT4Publisher());

            if (shouldWriteLogs)
                Logger.addDataReceiver(new WPILOGWriter(logPath));
        } else {
            robot.setUseTiming(true);
            logPath = LogFileUtil.findReplayLog();

            final String logWriterPath = LogFileUtil.addPathSuffix(logPath, "_replay");

            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(logWriterPath));
        }

        Logger.start();
    }


    /**
     * Add hardware device to hardware logging manager
     * <p>
     * Should not be necessary to call this manually, all devices register themselves when instantiated
     *
     * @param devices Devices to add
     */
    public static void addHardware(LoggableHardware... devices) {
        hardware.addAll(Arrays.asList(devices));
    }

    /**
     * Add custom periodicRunnable to the hardware logging manager to be called every loop
     *
     * @param periodicRunnable Desired periodicRunnable
     */
    public static void addCallback(Runnable periodicRunnable) {
        HardwareManager.periodicRunnable.add(periodicRunnable);
    }

    /**
     * Update all hardware devices
     * <p>
     * Call this periodically, preferably in the beginning of <code>robotPeriodic()</code> every loop
     */
    public static void update() {
        //TODO: Only if hardware should use lock, USE LOCK
        FASTER_THREAD_LOCK.lock();
        for (LoggableHardware loggableHardware : hardware) {
            loggableHardware.periodic();
        }
        FASTER_THREAD_LOCK.unlock();

        periodicRunnable.forEach(Runnable::run);
    }

    public static void updateSimulation() {
        GenericSimulation.updateAllSimulations();
    }
}
