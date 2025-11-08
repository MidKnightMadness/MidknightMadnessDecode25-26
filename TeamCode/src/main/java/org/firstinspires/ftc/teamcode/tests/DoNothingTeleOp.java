package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Do Nothing", group = "Competition")
public class DoNothingTeleOp extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addLine("Hi");
        telemetry.update();
    }
}
