package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Turret Test", group = "General")
public class TurretTest extends OpMode {
    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, false);

    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()){
            turret.setServos(0);
        }
        if(gamepad1.rightBumperWasPressed()){
            turret.setServos(1);
        }

        if(gamepad1.left_trigger > 0.5){
            turret.resetEncoderPosition();
        }

        if(gamepad1.right_trigger > 0.5){
            turret.setServos(0.5);
        }


    }
}
