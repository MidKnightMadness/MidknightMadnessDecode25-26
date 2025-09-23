package org.firstinspires.ftc.teamcode.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group="Motors")
public class MotorTest extends OpMode {
    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "BR");
    }

    @Override
    public void loop() {
        motor.setPower(0.5);
    }
}
