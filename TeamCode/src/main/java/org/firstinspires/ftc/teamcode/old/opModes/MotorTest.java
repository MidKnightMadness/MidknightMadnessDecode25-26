package org.firstinspires.ftc.teamcode.old.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp
public class MotorTest extends OpMode {
    DcMotorEx motor;


    // Front right is actually back right
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "FR");
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
