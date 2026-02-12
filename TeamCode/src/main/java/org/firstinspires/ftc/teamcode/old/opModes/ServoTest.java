package org.firstinspires.ftc.teamcode.old.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.robotDrive.WheelControl;
import org.firstinspires.ftc.teamcode.subsystems.ServoController;


@TeleOp
@Disabled
public class ServoTest extends OpMode {
    WheelControl drive;
    double forward;
    double leftRight;
    double angle;
    double power;
    boolean thing;
    ServoController servocontrol;
    Servo servo;

    @Override
    public void init() {
        drive = new WheelControl(hardwareMap);
        power = 1;
        thing = true;
        servo = hardwareMap.get(Servo.class, "servo");
        servocontrol = new ServoController(servo, -0.5, 0.5);
        servocontrol.setAngle(0);
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        leftRight = gamepad1.left_stick_x;
        angle = -gamepad1.right_stick_x;
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
