package frc.lib.generic.sensors;

public class SensorSignal {
    public enum SignalType {
        BEAM_BREAK(0);

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

    public SensorSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public SensorSignal(SignalType type) {
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
