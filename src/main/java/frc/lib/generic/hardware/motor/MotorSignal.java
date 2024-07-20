package frc.lib.generic.hardware.motor;

public class MotorSignal {
    public enum SignalType {
        CURRENT(1), POSITION(4), VELOCITY(5), VOLTAGE(0), TEMPERATURE(2), CLOSED_LOOP_TARGET(3);

        final int id;

        SignalType(int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }
    }

    private final SignalType type;
    private final boolean useFasterThread;

    public MotorSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public MotorSignal(SignalType type) {
        this(type, false);
    }

    public SignalType getType() {
        return type;
    }

    public String getName() {
        return type.name();
    }

    public double getUpdateRate() {
        return useFasterThread() ? 200 : 50;
    }

    public boolean useFasterThread() {
        return useFasterThread;
    }
}
