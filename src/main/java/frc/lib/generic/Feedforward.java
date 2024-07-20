package frc.lib.generic;

public class Feedforward {
    private final double kS; //Volts required to overcome the force of static friction
    private final double kV; //Volts required to maintain a velocity of one unit
    private final double kA; //Volts required to accelerate the mechanism by one unit
    private final double kG; //Volts required to overcome the force of gravity

    private final Properties.FeedforwardType type;

    public Feedforward(Properties.FeedforwardType type, double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = 0;

        this.type = type;
    }

    public Feedforward(Properties.FeedforwardType type, double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;

        this.type = type;
    }

    /**
     * @param positionRotations            For arm ONLY. This should be in rotations. For non-arms, this will be ignored.
     * @param velocityRotationsPerSec      unit-less, preferably in rotations per second
     * @param accelerationRotPerSecSquared unit-less, preferably rotations per second squared
     * @return The input for the motor, in voltage
     */
    public double calculate(double positionRotations, double velocityRotationsPerSec, double accelerationRotPerSecSquared) {
        switch (type) {
            case ARM -> {
                return calculateArmFeedforward(positionRotations, velocityRotationsPerSec, accelerationRotPerSecSquared);
            }

            case ELEVATOR -> {
                return calculateElevatorFeedforward(velocityRotationsPerSec, accelerationRotPerSecSquared);
            }

            case SIMPLE -> {
                return calculateSimpleMotorFeedforward(velocityRotationsPerSec, accelerationRotPerSecSquared);
            }
        }

        throw new UnsupportedOperationException("Can't seem to have a type. How is this even possible?");
    }

    /**
     * For both simple motor and elevator feed forwards. No position is required.
     *
     * @param velocity     - unit-less, preferably in rotations per second
     * @param acceleration - unit-less, preferably rotations per second squared
     * @return - the input for the motors
     */
    public double calculate(double velocity, double acceleration) {
        return calculate(0, velocity, acceleration);
    }

    /**
     * For both simple motor and elevator feed forwards. With no position or acceleration
     *
     * @param velocity - unit-less, preferably in rotations per second
     * @return - the input for the motors
     */
    public double calculate(double velocity) {
        return calculate(0, velocity, 0);
    }

    private double calculateSimpleMotorFeedforward(double velocity, double acceleration) {
        return kS * Math.signum(velocity)
                + kV * velocity
                + kA * acceleration;
    }

    private double calculateElevatorFeedforward(double velocity, double acceleration) {
        return kG
                + kS * Math.signum(velocity)
                + kV * velocity
                + kA * acceleration;
    }

    private double calculateArmFeedforward(double positionRadians, double velocity, double acceleration) {
        return kG * Math.cos(positionRadians * 2 * Math.PI)
                + kS * Math.signum(velocity)
                + kV * velocity
                + kA * acceleration;
    }
}
