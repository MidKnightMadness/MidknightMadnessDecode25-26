package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.MainTeleOpTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;

@Autonomous
public class DoNothingAuto extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        MainTeleOpTurret.startPoseX = 10.0;
        MainTeleOpTurret.startPoseY  = 10.0;
        MainTeleOpTurret.startPoseHeading = Math.PI;

        ConstantsBot.side = ShootSide.LEFT;
    }
}
