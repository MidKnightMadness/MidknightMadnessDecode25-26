package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
@Configurable
public class  GobildaDistance {
    AnalogInput analogInput;
    DigitalChannel digitalInput;
    RangerMode mode;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    public GobildaDistance(HardwareMap hardwareMap, String name, RangerMode mode) {
        if (mode == RangerMode.DIGITAL) {
            digitalInput = hardwareMap.get(DigitalChannel.class, name);
            digitalInput.setMode(DigitalChannel.Mode.INPUT);
        } else {
            analogInput = hardwareMap.get(AnalogInput.class, name);
        }

        this.mode = mode;
    }

    public double getDistance() {
//        switch (mode) {
//            case DEG15:
//                return analogInput.getVoltage() * 32.5 - 2.6;
//            case DEG20:
//                return analogInput.getVoltage() * 48.7 - 4.9;
//            case DEG27:
//                return analogInput.getVoltage() * 78.1 - 10.2;
//            case DIGITAL:
//                return 0;
//        }
        double volts = analogInput.getVoltage();
        double distanceInch = (volts / MAX_VOLTS) * MAX_DISTANCE_MM / 25.4;
        return distanceInch;
    }

    public boolean getState() {
        if (mode != RangerMode.DIGITAL) return false;

        return digitalInput.getState();
    }
}
