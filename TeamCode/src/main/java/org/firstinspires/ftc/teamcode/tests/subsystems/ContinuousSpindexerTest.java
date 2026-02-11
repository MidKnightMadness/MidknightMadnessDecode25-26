package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "Continuous Spindexer Test", group = "Spindexer")
public class ContinuousSpindexerTest extends OpMode {
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