package org.firstinspires.ftc.teamcode.main.autonomous;

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

    @Override
    public void initialize() {

    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void run() {
        startPoseX += 1/40d;
        startPoseY += 1/40d;
        startPoseHeading += Math.toRadians(1/40d);

        MainTeleOpTurret.startPoseX = startPoseX;
        MainTeleOpTurret.startPoseY  = startPoseY;
        MainTeleOpTurret.startPoseHeading = startPoseHeading;

        ConstantsBot.side = ShootSide.LEFT;

        telemetry.addLine(String.format("I will write: (%.2f, %.2f, %.2f)", startPoseX, startPoseY, startPoseHeading));
    }

    @Override
    public void end() {
        MainTeleOpTurret.startPoseX = startPoseX;
        MainTeleOpTurret.startPoseY  = startPoseY;
        MainTeleOpTurret.startPoseHeading = startPoseHeading;
    }
}
