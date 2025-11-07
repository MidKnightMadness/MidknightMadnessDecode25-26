package org.firstinspires.ftc.teamcode.Experiments;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DriveTestingIdk extends OpMode {
    WheelControlTemporaryMaybe drive;
    double forward;
    double leftRight;
    double angle;
    double power;
    boolean thing;

    @Override
    public void init() {
        drive = new WheelControlTemporaryMaybe(hardwareMap);
        power = 1;
        thing = true;
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        leftRight = gamepad1.left_stick_x;
        angle = gamepad1.right_stick_x;
        drive.drive_relative(forward, leftRight, angle, power);
        if(gamepad1.leftBumperWasPressed()){
            if (thing){
                power = 0.5;
                thing = false;
            }
            else{
                power = 1;
                thing = true;
            }
        }
    }
}
