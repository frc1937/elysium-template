package frc.lib.generic.hardware.motor.hardware;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.generic.Feedforward;
import frc.lib.generic.OdometryThread;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.hardware.motor.MotorSignal;
import frc.lib.math.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.motor.MotorInputs.MOTOR_INPUTS_LENGTH;

public class GenericSpark extends Motor {
    private final CANSparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkPIDController sparkController;

    private final boolean[] signalsToLog = new boolean[MOTOR_INPUTS_LENGTH];
    private final Map<String, Queue<Double>> signalQueueList = new HashMap<>();

    private double closedLoopTarget;

    private MotorConfiguration currentConfiguration;
    private Feedforward.Type feedforward;

    private int slotToUse = 0;
    private double conversionFactor = 1;

    private DoubleSupplier externalPositionSupplier, externalVelocitySupplier;

    private TrapezoidProfile positionMotionProfile, velocityMotionProfile;
    private TrapezoidProfile.State previousSetpoint, goalState;

    private double previousVelocity = 0;
    private boolean hasStoppedOccurred = false;
    private double lastProfileCalculationTimestamp;

    private final Timer timer = new Timer();

    public GenericSpark(String name, int deviceId, MotorProperties.SparkType sparkType) {
        super(name);

        spark = sparkType.sparkCreator.apply(deviceId);

        encoder = spark.getEncoder();
        sparkController = spark.getPIDController();

        optimizeBusUsage();

        timer.start();

        encoder.setPosition(getEffectivePosition());
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        setOutput(controlMode, output, 0);
    }

    @Override
    public void setOutput(MotorProperties.ControlMode mode, double output, double feedforward) {
        closedLoopTarget = output;
        setGoal(mode, output);

        switch (mode) {
            case PERCENTAGE_OUTPUT -> sparkController.setReference(output, CANSparkBase.ControlType.kDutyCycle);

            case POSITION, VELOCITY -> handleSmoothMotion(mode);

            case VOLTAGE -> sparkController.setReference(output, CANSparkBase.ControlType.kVoltage, slotToUse, 0);
            case CURRENT -> sparkController.setReference(output, CANSparkBase.ControlType.kCurrent, slotToUse, 0);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        spark.setIdleMode(idleMode == MotorProperties.IdleMode.COAST ? CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
    }

    @Override
    public void stopMotor() {
        hasStoppedOccurred = true;
        spark.stopMotor();
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public int getDeviceID() {
        return spark.getDeviceId();
    }

    @Override
    public void setExternalPositionSupplier(DoubleSupplier positionSupplier) {
        this.externalPositionSupplier = positionSupplier;
    }

    @Override
    public void setExternalVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.externalVelocitySupplier = velocitySupplier;
    }

    @Override
    public MotorConfiguration getCurrentConfiguration() {
        return currentConfiguration;
    }

    @Override
    public void setFollowerOf(String name, int masterPort) {
        spark.follow(new CANSparkMax(masterPort, CANSparkLowLevel.MotorType.kBrushless));
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10);
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        currentConfiguration = configuration;

        spark.restoreFactoryDefaults();

        setIdleMode(configuration.idleMode);
        spark.setInverted(configuration.inverted);

        spark.enableVoltageCompensation(12);

        encoder.setPositionConversionFactor(1 / configuration.gearRatio);
        encoder.setVelocityConversionFactor(1/ (configuration.gearRatio));

        conversionFactor = (1.0 / configuration.gearRatio);

        if (configuration.statorCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.statorCurrentLimit);
        if (configuration.supplyCurrentLimit != -1) spark.setSmartCurrentLimit((int) configuration.supplyCurrentLimit);

        configureProfile(configuration);
        configurePID(configuration);

        configureFeedForward();

        encoder.setPosition(getEffectivePosition());

        return spark.burnFlash() == REVLibError.kOk;
    }

    private void configureProfile(MotorConfiguration configuration) {
        if (configuration.profiledMaxVelocity != 0 && configuration.profiledTargetAcceleration != 0) {
            final TrapezoidProfile.Constraints positionMotionConstraints = new TrapezoidProfile.Constraints(
                    configuration.profiledMaxVelocity,
                    configuration.profiledTargetAcceleration
            );

            positionMotionProfile = new TrapezoidProfile(positionMotionConstraints);
        }

        if (configuration.profiledTargetAcceleration != 0 && configuration.profiledJerk != 0) {
            final TrapezoidProfile.Constraints velocityMotionConstraints = new TrapezoidProfile.Constraints(
                    configuration.profiledTargetAcceleration,
                    configuration.profiledJerk
            );

            velocityMotionProfile = new TrapezoidProfile(velocityMotionConstraints);
        }
    }

    private void configureFeedForward() {
        final MotorProperties.Slot currentSlot = getCurrentSlot();

        if (currentSlot.gravityType() == null) feedforward = Feedforward.Type.SIMPLE;

        if (currentSlot.gravityType() == GravityTypeValue.Arm_Cosine) feedforward = Feedforward.Type.ARM;

        if (currentSlot.gravityType() == GravityTypeValue.Elevator_Static) feedforward = Feedforward.Type.ELEVATOR;

        feedforward.setFeedforwardConstants(
                currentSlot.kS(),
                currentSlot.kV(),
                currentSlot.kA(),
                currentSlot.kG()
        );
    }

    private void configurePID(MotorConfiguration configuration) {
        slotToUse = configuration.slotToUse;

        sparkController.setP(configuration.slot0.kP(), 0);
        sparkController.setI(configuration.slot0.kI(), 0);
        sparkController.setD(configuration.slot0.kD(), 0);

        sparkController.setP(configuration.slot1.kP(), 1);
        sparkController.setI(configuration.slot1.kI(), 1);
        sparkController.setD(configuration.slot1.kD(), 1);

        sparkController.setP(configuration.slot2.kP(), 2);
        sparkController.setI(configuration.slot2.kI(), 2);
        sparkController.setD(configuration.slot2.kD(), 2);

        sparkController.setPositionPIDWrappingEnabled(configuration.closedLoopContinuousWrap);
        sparkController.setSmartMotionAllowedClosedLoopError(configuration.closedLoopTolerance, configuration.slotToUse);
        //check if works, and if theres a default
    }

    /**
     * Set all other status to basically never(10sec) to optimize bus usage
     * Only call this ONCE at the beginning.
     */
    private void optimizeBusUsage() {
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767);
        spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, 32767);
    }

    private void handleSmoothMotion(MotorProperties.ControlMode controlMode) {
        double feedforwardOutput, acceleration;
        final CANSparkBase.ControlType controlType = controlMode == MotorProperties.ControlMode.POSITION ? CANSparkBase.ControlType.kPosition : CANSparkBase.ControlType.kVelocity;

        if (positionMotionProfile != null && controlMode == MotorProperties.ControlMode.POSITION) {
            if (goalState == null) return;

            final TrapezoidProfile.State currentSetpoint = positionMotionProfile.calculate(0.02, previousSetpoint, goalState);

            acceleration = (currentSetpoint.velocity - previousSetpoint.velocity) / 0.02;

            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity, acceleration);
            //set the pos of sparkController getEffectivePosition

            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getTimestamp();

            sparkController.setReference(currentSetpoint.position,
                    controlType, slotToUse, feedforwardOutput,
                    SparkPIDController.ArbFFUnits.kVoltage);
        } else if (velocityMotionProfile != null && controlMode == MotorProperties.ControlMode.VELOCITY) {
            if (goalState == null) return;

            final TrapezoidProfile.State currentSetpoint = velocityMotionProfile.calculate(0.02, previousSetpoint, goalState);

            feedforwardOutput = this.feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity);
            //set the vel of sparkController getEffectiveVelocity

            previousSetpoint = currentSetpoint;
            lastProfileCalculationTimestamp = Logger.getRealTimestamp();

            //this is position because pos is velocity on upgraded
            sparkController.setReference(currentSetpoint.position * 60, controlType, slotToUse, feedforwardOutput,
                    SparkPIDController.ArbFFUnits.kVoltage);
        } else {
            if (goalState == null) return;

            feedforwardOutput = this.feedforward.calculate(goalState.position, goalState.velocity, 0);

            final double goal = controlType == CANSparkBase.ControlType.kPosition ? goalState.position : 60 * goalState.position;

            sparkController.setReference(goal,
                    controlType, slotToUse, feedforwardOutput,
                    SparkPIDController.ArbFFUnits.kVoltage);
        }
    }

    private void setGoal(MotorProperties.ControlMode controlMode, double goal) {
        if (!shouldResetProfile(new TrapezoidProfile.State(goal, 0))) return;

        hasStoppedOccurred = false;

        if (controlMode == MotorProperties.ControlMode.POSITION)
            previousSetpoint = new TrapezoidProfile.State(getEffectivePosition(), getEffectiveVelocity());
        else if (controlMode == MotorProperties.ControlMode.VELOCITY)
            previousSetpoint = new TrapezoidProfile.State(getEffectiveVelocity(), getEffectiveAcceleration());

        goalState = new TrapezoidProfile.State(goal, 0);
    }

    private double getEffectivePosition() {
        return externalPositionSupplier == null ? getSystemPositionPrivate() : externalPositionSupplier.getAsDouble();
    }

    private double getEffectiveVelocity() {
        return externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();
    }

    private double getEffectiveAcceleration() {
        final double acceleration = externalVelocitySupplier == null ? getSystemVelocityPrivate() - previousVelocity : externalVelocitySupplier.getAsDouble() - previousVelocity;

        previousVelocity = externalVelocitySupplier == null ? getSystemVelocityPrivate() : externalVelocitySupplier.getAsDouble();

        return acceleration;
    }

    /**
     * Explanation here: <a href="https://docs.revrobotics.com/brushless/spark-max/control-interfaces">REV DOCS</a>
     */
    @Override
    public void setupSignalUpdates(MotorSignal signal, boolean useFasterThread) {
        final int ms = 1000 / (useFasterThread ? 200 : 50);

        signalsToLog[signal.getId()] = true;

        switch (signal) {
            case VELOCITY, CURRENT, TEMPERATURE ->
                    spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, ms);
            case POSITION -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, ms);
            case VOLTAGE -> spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, ms);
        }

        if (!useFasterThread) return;

        signalsToLog[signal.getId() + MOTOR_INPUTS_LENGTH / 2] = true;

        switch (signal) {
            case POSITION ->
                    signalQueueList.put("position", OdometryThread.getInstance().registerSignal(this::getSystemPositionPrivate));
            case VELOCITY ->
                    signalQueueList.put("velocity", OdometryThread.getInstance().registerSignal(this::getSystemVelocityPrivate));
            case CURRENT ->
                    signalQueueList.put("current", OdometryThread.getInstance().registerSignal(spark::getOutputCurrent));
            case VOLTAGE ->
                    signalQueueList.put("voltage", OdometryThread.getInstance().registerSignal(this::getVoltagePrivate));
            case TEMPERATURE ->
                    signalQueueList.put("temperature", OdometryThread.getInstance().registerSignal(spark::getMotorTemperature));
            case CLOSED_LOOP_TARGET ->
                    signalQueueList.put("target", OdometryThread.getInstance().registerSignal(() -> closedLoopTarget));
        }
    }

    @Override
    protected boolean[] getSignalsToLog() {
        return signalsToLog;
    }

    @Override
    protected void refreshInputs(MotorInputs inputs) {
        if (spark == null) return;

        if (timer.advanceIfElapsed(1)) {
            encoder.setPosition(getEffectivePosition());
        }

        inputs.setSignalsToLog(signalsToLog);

        if (signalsToLog[0]) inputs.voltage = getVoltagePrivate();
        if (signalsToLog[1]) inputs.current = spark.getOutputCurrent();
        if (signalsToLog[2]) inputs.temperature = spark.getMotorTemperature();
        if (signalsToLog[3]) inputs.target = closedLoopTarget;
        if (signalsToLog[4]) inputs.systemPosition = getEffectivePosition();
        if (signalsToLog[5]) inputs.systemVelocity = getEffectiveVelocity();

        MotorUtilities.handleThreadedInputs(inputs, signalQueueList);
    }

    private double getVoltagePrivate() {
        return spark.getBusVoltage() * spark.getAppliedOutput();
    }

    private double getSystemPositionPrivate() {
        return encoder.getPosition() * conversionFactor;
    }

    private double getSystemVelocityPrivate() {
        return (encoder.getVelocity() / Conversions.SEC_PER_MIN) * conversionFactor;
    }

    private boolean shouldResetProfile(TrapezoidProfile.State newGoal) {
        return goalState == null
                || !goalState.equals(newGoal)
                || hasStoppedOccurred
                || (Logger.getRealTimestamp() - lastProfileCalculationTimestamp) > 100000; //(0.1 sec has passed)
    }
}
