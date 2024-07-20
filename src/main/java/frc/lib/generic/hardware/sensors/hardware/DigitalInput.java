package frc.lib.generic.hardware.sensors.hardware;

import frc.lib.generic.hardware.sensors.SensorInputs;
import frc.lib.generic.hardware.sensors.Sensor;

public class DigitalInput extends Sensor {
    private final edu.wpi.first.wpilibj.DigitalInput digitalInput;

    public DigitalInput(String name, int id) {
        super(name);

        digitalInput = new edu.wpi.first.wpilibj.DigitalInput(id);
    }

//todo: Implement this fucktarded sensor kus emek

    @Override
    public void refreshInputs(SensorInputs inputs) {
        if (digitalInput == null) return;

        inputs.currentValue = digitalInput.get() ? 1 : 0;

        //todo: add support for faster akit inputs
    }
}
