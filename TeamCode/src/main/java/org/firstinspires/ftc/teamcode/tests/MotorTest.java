package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "motor test", group = "General")
@Disabled
public class MotorTest extends OpMode {
    DcMotorEx FR;


    @Override
    public void init() {
        FR = hardwareMap.get(DcMotorEx.class, ConfigNames.FR);
    }

    @Override
    public void loop() {
        FR.setPower(1);
    }
}
