package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl;

@TeleOp
@Disabled
public class BasicDrive extends OpMode {
    WheelControl wheelControl;

    @Override
    public void init() {
        wheelControl = new WheelControl(hardwareMap);
    }

    @Override
    public void loop() {
        wheelControl.driveRelative (-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, 1);
    }
}
