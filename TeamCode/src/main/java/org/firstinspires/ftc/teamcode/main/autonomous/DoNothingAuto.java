package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.MainTeleOpTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;

@Autonomous
public class DoNothingAuto extends OpMode {

    double startPoseX = 10;
    double startPoseY = 1;

    double startPoseHeading = Math.PI;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        startPoseX += 1/40d;
        startPoseY += 1/40d;
        startPoseHeading += Math.toRadians(1/40d);

        MainTeleOpTurret.startPoseX = startPoseX;
        MainTeleOpTurret.startPoseY  = startPoseY;
        MainTeleOpTurret.startPoseHeading = startPoseHeading;



        ConstantsBot.side = ShootSide.LEFT;
    }
}
