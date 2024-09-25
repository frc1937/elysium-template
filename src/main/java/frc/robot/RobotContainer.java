// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.util.Controller;
import frc.robot.commands.ShooterCommands;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.robot.GlobalConstants.BLUE_SPEAKER;
import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.FRONT_CAMERA;

public class RobotContainer {
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(FRONT_CAMERA);
    public static final Swerve SWERVE = new Swerve();
    public static final Arm ARM = new Arm();
    public static final Flywheels FLYWHEELS = new Flywheels();
    public static final Intake INTAKE = new Intake();
    public static final Kicker KICKER = new Kicker();
    public static final Leds LEDS = new Leds();

    public static final BuiltInAccelerometer ROBORIO_ACCELEROMETER = new BuiltInAccelerometer();

    private final ShooterCommands shooterCommands = new ShooterCommands();

    private final Trigger userButton = new Trigger(RobotController::getUserButton);

    private final Controller driveController = new Controller(0);

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser("Teared down Crown"));

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        setupLEDs();

        DoubleSupplier translationSupplier = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSupplier = () -> -driveController.getRawAxis(LEFT_X);

        SWERVE.setDefaultCommand(
                SWERVE.driveOpenLoop(
                        translationSupplier,
                        strafeSupplier,

                        () -> -driveController.getRawAxis(Controller.Axis.RIGHT_X),
                        () -> driveController.getStick(Controller.Stick.RIGHT_STICK).getAsBoolean()
                ));

        driveController.getButton(Controller.Inputs.START).whileTrue(SWERVE.resetGyro());
        driveController.getButton(Controller.Inputs.BACK).whileTrue(SWERVE.lockSwerve());

//        driveController.getButton(Controller.Inputs.A)
//                .whileTrue(SWERVE.driveWhilstRotatingToTarget(translationSupplier, strafeSupplier,
//                                BLUE_SPEAKER.toPose2d(), () -> false)
//                        .alongWith(shooterCommands.shootPhysics(BLUE_SPEAKER, 15))
//                );

//        driveController.getButton(Controller.Inputs.A).whileTrue(
//                ARM.setTargetPosition(Rotation2d.fromDegrees(60)).alongWith(FLYWHEELS.setTargetVelocity(50))
//                        .alongWith(new WaitCommand(3).andThen(KICKER.setKickerPercentageOutput(1))
//                )
//        );

//        driveController.getButton(Controller.Inputs.A).whileTrue(ARM.setTargetPosition(Rotation2d.fromDegrees(30)));
//        driveController.getButton(Controller.Inputs.B).whileTrue(ARM.setTargetPosition(Rotation2d.fromDegrees(60)));
//        driveController.getButton(Controller.Inputs.Y).whileTrue(ARM.setTargetPosition(Rotation2d.fromDegrees(90)));
//        driveController.getButton(Controller.Inputs.X).whileTrue(ARM.setTargetPosition(Rotation2d.fromDegrees(-10)));

        driveController.getButton(Controller.Inputs.A).whileTrue(FLYWHEELS.setTargetVelocity(15));
        driveController.getButton(Controller.Inputs.B).whileTrue(FLYWHEELS.setTargetVelocity(30));
        driveController.getButton(Controller.Inputs.Y).whileTrue(FLYWHEELS.setTargetVelocity(-10));
        driveController.getButton(Controller.Inputs.X).whileTrue(FLYWHEELS.setTargetVelocity(-5));


        driveController.getStick(Controller.Stick.LEFT_STICK).whileTrue(shooterCommands.receiveFloorNote());
        driveController.getButton(Controller.Inputs.LEFT_BUMPER).whileTrue(shooterCommands.outtakeNote());

        driveController.getStick(Controller.Stick.RIGHT_STICK).whileTrue(
                shooterCommands.shootPhysics(BLUE_SPEAKER, 11));

        driveController.getButton(Controller.Inputs.RIGHT_BUMPER).whileTrue(
                SWERVE.driveWhilstRotatingToTarget(() -> 0, () -> 0,
                        BLUE_SPEAKER.toPose2d(), () -> false)
        );

        userButton.toggleOnTrue(Commands.startEnd(
                () -> {
                    ARM.setIdleMode(MotorProperties.IdleMode.COAST);
                    LEDS.setLEDStatus(Leds.LEDMode.SHOOTER_EMPTY, 15);
                },

                () -> ARM.setIdleMode(MotorProperties.IdleMode.BRAKE),

                ARM, LEDS).ignoringDisable(true)
        ).debounce(0.5);

        configureButtons(ButtonLayout.TELEOP);
    }

    private void configureButtons(ButtonLayout layout) {
        switch (layout) {
            case CHARACTERIZE_ARM -> setupCharacterization(ARM);
            case CHARACTERIZE_FLYWHEEL -> setupCharacterization(FLYWHEELS);
        }
    }

    private void setupCharacterization(GenericSubsystem subsystem) {
        driveController.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        driveController.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        driveController.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private enum ButtonLayout {
        TELEOP,
        CHARACTERIZE_FLYWHEEL,
        CHARACTERIZE_ARM
    }

    private void setupLEDs() {
        LEDS.setDefaultCommand(LEDS.setLEDStatus(Leds.LEDMode.DEFAULT, 0));

        final int LOW_BATTERY_THRESHOLD = 150;
        final int[] lowBatteryCounter = {0};

        new Trigger(() -> {
            if (RobotController.getBatteryVoltage() < 11.7) lowBatteryCounter[0]++;

            return LOW_BATTERY_THRESHOLD < lowBatteryCounter[0];
        }).onTrue(LEDS.setLEDStatus(Leds.LEDMode.BATTERY_LOW, 10));

        new Trigger(KICKER::doesSeeNote).onTrue(LEDS.setLEDStatus(Leds.LEDMode.SHOOTER_EMPTY, 3));

    }
}
