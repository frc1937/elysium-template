package frc.lib.generic.motor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.arm.real.RealArmConstants.ABSOLUTE_ARM_ENCODER;

public class PurpleSpark extends CANSparkBase implements Motor {
    private final MotorProperties.SparkType model;
    private final RelativeEncoder encoder;
    private final SparkPIDController controller;

    private MotorConfiguration currentConfiguration;
    private double closedLoopTarget;
    private Feedforward feedforward;

    private int slotToUse = 0;

    private PIDController feedback;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State previousSetpoint;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();

    private double previousTimestamp = Logger.getTimestamp();

    public PurpleSpark(int deviceId, MotorProperties.SparkType sparkType) {
        super(deviceId, MotorType.kBrushless, sparkType == MotorProperties.SparkType.MAX ? SparkModel.SparkMax : SparkModel.SparkFlex);
        model = sparkType;

        optimizeBusUsage();

        encoder = this.getEncoder();
        controller = super.getPIDController();
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0.0);
        //todo: fix this to ACTUALLY use the FF provided here.
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);

        switch (mode) {
            case PERCENTAGE_OUTPUT -> set(output);

            case VELOCITY -> controller.setReference(output * 60, ControlType.kVelocity, slotToUse, feedforward);
            case POSITION -> handleSmoothMotion();

            case VOLTAGE -> setVoltage(output);
            case CURRENT -> controller.setReference(output, ControlType.kCurrent, slotToUse);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? IdleMode.kCoast : IdleMode.kBrake);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return getDeviceId();
    }

    @Override
    public MotorProperties.Slot getCurrentSlot() {
        return getSlot(slotToUse, currentConfiguration);
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void resetSlot(MotorProperties.Slot slot, int slotNumber) {
        switch (slotNumber) {
            case 0 -> currentConfiguration.slot0 = slot;
            case 1 -> currentConfiguration.slot1 = slot;
            case 2 -> currentConfiguration.slot2 = slot;
        }

        configure(currentConfiguration);
    }

    @Override
    public double getMotorPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getMotorVelocity() {
        return encoder.getVelocity() / 60;
    }

    @Override
    public double getCurrent() {
        return super.getOutputCurrent();
    }

    @Override
    public double getTemperature() {
        return getMotorTemperature();
    }

    @Override
    public double getSystemPosition() {
        return ABSOLUTE_ARM_ENCODER.getEncoderPosition();// encoder.getPosition(); /// currentConfiguration.gearRatio;
    }

    @Override
    public double getSystemVelocity() {
        //todo: use ACTUAL relative ENCODERS...
        return ABSOLUTE_ARM_ENCODER.getEncoderVelocity();//encoder.getVelocity() / (60 * currentConfiguration.gearRatio);
    }

    @Override
    public double getClosedLoopTarget() {
        return closedLoopTarget;
    }

    @Override
    public double getVoltage() {
        return super.getBusVoltage() * getAppliedOutput();
    }

    @Override
    public void setFollowerOf(int masterPort) {
        super.follow(new GenericSpark(masterPort, model));
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames">REV DOCS</a>
     */
    @Override
    public void setSignalUpdateFrequency(Properties.SignalType signalType, double updateFrequencyHz) {
        int ms = (int) (1000 / updateFrequencyHz);

        switch (signalType) {
            case VELOCITY, CURRENT -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ms);
            case POSITION -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, ms);
        }
    }

    @Override
    public void setSignalsUpdateFrequency(double updateFrequencyHz, Properties.SignalType... signalTypes) {
        for (Properties.SignalType signalType : signalTypes) {
            setSignalUpdateFrequency(signalType, updateFrequencyHz);
        }
    }

    @Override
    public StatusSignal<Double> getRawStatusSignal(Properties.SignalType signalType) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public void refreshStatusSignals(Properties.SignalType... signalTypes) {
        throw new UnsupportedOperationException("SparkMaxes don't use status signals. This operation is not supported.");
    }

    @Override
    public TalonFXSimState getSimulationState() {
        throw new UnsupportedOperationException("Not supported. Use GenericTalonFX");
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        super.restoreFactoryDefaults();

        super.setIdleMode(configuration.idleMode.equals(MotorProperties.IdleMode.BRAKE) ? IdleMode.kBrake : IdleMode.kCoast);
        super.setInverted(configuration.inverted);

        super.enableVoltageCompensation(12);

        if (configuration.statorCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) super.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);

        configurePID(configuration);
        configureFeedForward(configuration);

        return super.burnFlash() == REVLibError.kOk;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity == 0 || configuration.profiledTargetAcceleration == 0) return;

        final TrapezoidProfile.Constraints motionConstraints = new TrapezoidProfile.Constraints(
                configuration.profiledMaxVelocity,
                configuration.profiledTargetAcceleration
        );

        motionProfile = new TrapezoidProfile(motionConstraints);

        controller.setSmartMotionMaxVelocity(configuration.profiledMaxVelocity / 60, slotToUse);
        controller.setSmartMotionMaxAccel(configuration.profiledTargetAcceleration / 60, slotToUse);

//        controller.setSmartMotionAllowedClosedLoopError(configuration.closedLoopError, slotToUse);//todo: Might be needed. do test.
        controller.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, slotToUse);
    }

    private void configureFeedForward(MotorConfiguration configuration) {
        MotorProperties.Slot currentSlot = configuration.slot0;

        switch (slotToUse) {
            case 1 -> currentSlot = configuration.slot1;
            case 2 -> currentSlot = configuration.slot2;
        }

        if (currentSlot.gravityType() == null) {
            feedforward = new Feedforward(Properties.FeedforwardType.SIMPLE,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Arm_Cosine) {
            feedforward = new Feedforward(Properties.FeedforwardType.ARM,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }

        if (currentSlot.gravityType() == GravityTypeValue.Elevator_Static) {
            feedforward = new Feedforward(Properties.FeedforwardType.ELEVATOR,
                    currentSlot.kS(),
                    currentSlot.kV(),
                    currentSlot.kA(),
                    currentSlot.kG()
            );
        }
    }

    private void configurePID(MotorConfiguration configuration) {
        controller.setPositionPIDWrappingEnabled(configuration.closedLoopContinuousWrap);

        controller.setP(configuration.slot0.kP(), 0);
        controller.setI(configuration.slot0.kI(), 0);
        controller.setD(configuration.slot0.kD(), 0);

        controller.setP(configuration.slot1.kP(), 1);
        controller.setI(configuration.slot1.kI(), 1);
        controller.setD(configuration.slot1.kD(), 1);

        controller.setP(configuration.slot2.kP(), 2);
        controller.setI(configuration.slot2.kI(), 2);
        controller.setD(configuration.slot2.kD(), 2);

        feedback = new PIDController(configuration.slot0.kP(), configuration.slot0.kI(), configuration.slot0.kD());

        slotToUse = configuration.slotToUse;
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
        super.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 10000);
    }

    @Override
    public RelativeEncoder getEncoder() {
        switch (model) {
            case MAX -> {
                return getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
            }

            case FLEX -> {
                return getEncoder(SparkRelativeEncoder.Type.kQuadrature, 7168);
            }
        }

        return null;
    }

    private void handleSmoothMotion() {
        if (motionProfile == null)
            return; //todo: Profile-less motion (For velocity: USE FF. for POSITION: DONT USE FF.)

        final double timeDifference = ((Logger.getTimestamp() - previousTimestamp) / 1000000);
        final TrapezoidProfile.State currentSetpoint = motionProfile.calculate(timeDifference, previousSetpoint, goalState);

        final double acceleration = currentSetpoint.velocity - previousSetpoint.velocity / timeDifference;

        final double feedforwardOutput = getFeedforwardOutput(goalState, acceleration);
        final double feedbackOutput = feedback.calculate(getSystemPosition(), currentSetpoint.position);

        super.setVoltage(feedforwardOutput + feedbackOutput);

        previousSetpoint = currentSetpoint;
        previousTimestamp = Logger.getTimestamp();

        Logger.recordOutput("ProfiledPOSITION", currentSetpoint.position * 360);
        Logger.recordOutput("ProfiledVELOCITY", currentSetpoint.velocity * 360);

        Logger.recordOutput("Profiled CURRENT POSITION", getSystemPosition() * 360);
        Logger.recordOutput("Profiled CURRENT VELOCITY", getSystemVelocity() * 360);

        Logger.recordOutput("FeeFF", feedforwardOutput);
        Logger.recordOutput("FeeFB", feedbackOutput);
        Logger.recordOutput("FeeVOLT", feedforwardOutput + feedbackOutput);
    }


    private void setGoal(MotorProperties.ControlMode controlMode, double output) {
        TrapezoidProfile.State stateFromOutput = null;

        if (controlMode == MotorProperties.ControlMode.POSITION) stateFromOutput = new TrapezoidProfile.State(output, 0);
        if (controlMode == MotorProperties.ControlMode.VELOCITY) stateFromOutput = new TrapezoidProfile.State(0, output);

        if (stateFromOutput != null && goalState == null || !goalState.equals(stateFromOutput)) {
            feedback.reset();

            previousSetpoint = new TrapezoidProfile.State(getSystemPosition(), getSystemVelocity());
            goalState = new TrapezoidProfile.State(output, 0.0);

            DriverStation.reportError("[PurpleSpark] goal has changed", false);
        }
    }

    private double getFeedforwardOutput(TrapezoidProfile.State goal, double acceleration) {
        switch (getCurrentSlot().gravityType()) {
            case Elevator_Static -> {
                return calculateElevatorFeedforward(goal, acceleration);
            }
            case Arm_Cosine -> {
                return calculateArmFeedforward(goal, acceleration);
            }

            default -> {
                return calculateSimpleMotorFeedforward(goal, acceleration);
            }
        }
    }

    private double calculateSimpleMotorFeedforward(TrapezoidProfile.State goal, double acceleration) {
        return getCurrentSlot().kV() * goal.velocity
                + getCurrentSlot().kS() * Math.signum(goal.velocity)
                + getCurrentSlot().kA() * acceleration;
    }

    private double calculateElevatorFeedforward(TrapezoidProfile.State goal, double acceleration) {
        return getCurrentSlot().kG()
                + getCurrentSlot().kV() * goal.velocity
                + getCurrentSlot().kS() * Math.signum(goal.velocity)
                + getCurrentSlot().kA() * acceleration;
    }

    //todo: move all of these to a separate class
    private double calculateArmFeedforward(TrapezoidProfile.State goal, double acceleration) {
        return getCurrentSlot().kG() * Math.cos(goal.position * 2 * Math.PI)
                + getCurrentSlot().kV() * goal.velocity
                + getCurrentSlot().kS() * Math.signum(goal.velocity)
                + getCurrentSlot().kA() * acceleration;
    }
}