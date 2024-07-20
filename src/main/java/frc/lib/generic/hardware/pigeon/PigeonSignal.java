package frc.lib.generic.hardware.pigeon;

public class PigeonSignal {
    public enum SignalType {
        YAW(0), PITCH(1), ROLL(2);

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

    public PigeonSignal(SignalType type, boolean useFasterThread) {
        this.type = type;
        this.useFasterThread = useFasterThread;
    }

    public PigeonSignal(SignalType type) {
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
