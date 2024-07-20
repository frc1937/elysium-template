package frc.lib.generic.hardware.pigeon;

import frc.lib.generic.hardware.pigeon.hardware.IMUPigeon;
import frc.lib.generic.hardware.pigeon.hardware.SimulatedIMU;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class PigeonFactory {
    public static Pigeon createIMU(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REPLAY) {
            return new Pigeon(name);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedIMU(name);
        }

        return new IMUPigeon(name, port);
    }
}
