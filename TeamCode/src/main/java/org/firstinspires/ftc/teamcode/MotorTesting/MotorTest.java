/*
package org.firstinspires.ftc.teamcode.MotorTesting;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp(group = "MotorTest")
public class MotorTest extends OpMode {
    public static String motorName = "motor";
    public static double motorPower = 1;
    private DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    @Override
    public void loop() {
        motor.setPower(motorPower);
    }
}
*/