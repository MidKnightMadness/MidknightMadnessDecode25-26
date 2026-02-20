package org.firstinspires.ftc.teamcode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Beam Break Test")
public class BeamBreakerTest extends OpMode {

    DigitalChannel beam;

    @Override
    public void init() {
        beam = hardwareMap.get(DigitalChannel.class, "breaker");
        beam.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {

        if (!beam.getState()) {
            telemetry.addLine("Object Detected");
        } else {
            telemetry.addLine("Beam Clear");
        }

        telemetry.update();
    }
}
