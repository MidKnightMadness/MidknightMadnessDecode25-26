package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "SpindexerBasicTest", group = "Spindexer")
public class SpindexerBasicTest extends OpMode {
    CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, ConfigNames.turner);
    }

    @Override
    public void loop() {
        servo.setPower(1);
    }
}