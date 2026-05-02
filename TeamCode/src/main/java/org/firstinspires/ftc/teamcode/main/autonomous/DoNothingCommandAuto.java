package org.firstinspires.ftc.teamcode.main.autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.MainTeleOpTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;

@Autonomous
public class DoNothingCommandAuto extends CommandOpMode {

    double startPoseX = 10;
    double startPoseY = 1;

    double startPoseHeading = Math.PI;

    Follower follower;

    @Override
    public void initialize() {
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void run() {
        follower.update();

        if (follower.getPose().getX() == 0 && follower.getPose().getY() == 0) {
            return;
        }
        startPoseX = follower.getPose().getX();
        startPoseY = follower.getPose().getY();
        startPoseHeading = follower.getPose().getHeading();

        MainTeleOpTurret.startPoseX = startPoseX;
        MainTeleOpTurret.startPoseY = startPoseY;
        MainTeleOpTurret.startPoseHeading = startPoseHeading;

        ConstantsBot.side = ShootSide.LEFT;

        telemetry.addLine(String.format("I will write: (%.2f, %.2f, %.2f)", startPoseX, startPoseY, startPoseHeading));
        telemetry.update();
    }

//    @Override
//    public void end() {
//        MainTeleOpTurret.startPoseX = startPoseX;
//        MainTeleOpTurret.startPoseY  = startPoseY;
//        MainTeleOpTurret.startPoseHeading = startPoseHeading;
//    }
}
