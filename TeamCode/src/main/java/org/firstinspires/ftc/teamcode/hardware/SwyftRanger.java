package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SwyftRanger {
    AnalogInput analogInput;
    DigitalChannel digitalInput;
    RangerMode mode;

    public SwyftRanger(HardwareMap hardwareMap, String name, RangerMode mode) {
        if (mode == RangerMode.DIGITAL) {
            digitalInput = hardwareMap.get(DigitalChannel.class, name);
            digitalInput.setMode(DigitalChannel.Mode.INPUT);
        } else {
            analogInput = hardwareMap.get(AnalogInput.class, name);
        }
    }

    public double getDistance() {
        switch (mode) {
            case DEG15:
                return analogInput.getVoltage() * 32.5 - 2.6;
            case DEG20:
                return analogInput.getVoltage() * 48.7 - 4.9;
            case DEG27:
                return analogInput.getVoltage() * 78.1 - 10.2;
            case DIGITAL:
                return 0;
        }

        return 0;
    }

    public boolean getState() {
        if (mode != RangerMode.DIGITAL) return false;

        return digitalInput.getState();
    }
}
