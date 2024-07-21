// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.poseestimation.poseestimator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

import static frc.robot.GlobalConstants.FASTER_THREAD_LOCK;
import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread extends Thread {
    private final List<DoubleSupplier> signals = new ArrayList<>();
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final Queue<Double> timestamps = new ArrayBlockingQueue<>(100);
    private static OdometryThread instance = null;

    public static OdometryThread getInstance() {
        if (instance == null) {
            instance = new OdometryThread();
        }
        return instance;
    }

    private OdometryThread() {
        Notifier notifier = new Notifier(this::periodic);
        notifier.setName("OdometryThread");
        Timer.delay(1);
        notifier.startPeriodic(1.0 / ODOMETRY_FREQUENCY_HERTZ);
    }

    public Queue<Double> getTimestampQueue() {
        return timestamps;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(100);
        FASTER_THREAD_LOCK.lock();

        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }

        return queue;
    }

    private void periodic() {
        FASTER_THREAD_LOCK.lock();
        timestamps.offer(Logger.getRealTimestamp() / 1.0e6);

        try {
            for (int i = 0; i < signals.size(); i++) {
                queues.get(i).offer(signals.get(i).getAsDouble());
            }
        } finally {
            FASTER_THREAD_LOCK.unlock();
        }
    }
}