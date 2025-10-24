package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.annotation.Annotation;

@Configurable
@TeleOp(name = "DriveTeleOp")
public class DriveTeleOp extends OpMode {
    private Follower follower;
    public static Pose startPose = new Pose(72, 203.2 /25.4, Math.toRadians(90));


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }
    public static boolean slowMode = false;
    public static double slowSpeed = 0.5;

    @Override
    public void start(){
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.update();
        if(!slowMode){
            follower.setTeleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_y, false);
        }
        else{
            follower.setTeleOpDrive(-gamepad1.left_stick_x * slowSpeed, gamepad1.left_stick_y * slowSpeed, -gamepad1.right_stick_y * slowSpeed, false);
        }

        if(gamepad1.yWasPressed()){
            slowSpeed += 0.03;
        }
        if (gamepad1.bWasPressed()) {
            slowSpeed -= 0.03;
        }

        if(gamepad1.rightBumperWasPressed()){
            slowMode = !slowMode;
        }

        telemetry.addData("Robot Pos", follower.getPose());
        telemetry.addData("Speed", slowSpeed);
        telemetry.addData("Slow Mode", slowMode);
    }
}
