// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.simulation.GenericSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        switch (CURRENT_MODE) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIMULATION:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();
        RobotContainer.POSE_ESTIMATOR.periodic();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {
//        final TalonFXConfiguration config = new TalonFXConfiguration();
//
//        config.Slot0.kP = P;
//        config.Slot0.kI = I;
//        config.Slot0.kD = D;
//        config.Slot0.kG = KG;
//        config.Slot0.kV = KV;
//        config.Slot0.kA = KA;
//        config.Slot0.kS = KS;
//        config.ClosedLoopGeneral.ContinuousWrap = true;
//        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
//
//        config.MotionMagic.MotionMagicCruiseVelocity = MAXIMUM_VELOCITY;
//        config.MotionMagic.MotionMagicAcceleration = MAXIMUM_ACCELERATION;
//
//        talonFX.getConfigurator().apply(config);
//
//        Timer.delay(0.5);
    }

    @Override
    public void simulationPeriodic() {
        GenericSimulation.updateAllSimulations();
    }

    @Override
    public void close() {
        super.close();
    }
}
