package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Do Nothing", group = "Competition")
public class DoNothingTeleOp extends OpMode {
    double loopNum = 0;
    @Override
    public void init() {
        telemetry.addLine("OpMode initialized");
    }

    @Override
    public void start(){
        telemetry.addLine("OpMode started");
    }
    @Override
    public void loop() {
        telemetry.addData("OpMode loop", loopNum);
        telemetry.update();
        loopNum++;
    }
}
