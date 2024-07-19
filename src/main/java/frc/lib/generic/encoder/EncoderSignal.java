package frc.lib.generic.encoder;

public class EncoderSignal {
    public enum SignalType {
        POSITION(0), VELOCITY(1);

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

    public EncoderSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public EncoderSignal(SignalType type) {
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
