package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
@TeleOp(name = "MotorTest", group = "General")
public class MotorTest extends OpMode {
    public static String motorName = "FR";
    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    @Override
    public void loop() {
        motor.setPower(1);
    }
}
