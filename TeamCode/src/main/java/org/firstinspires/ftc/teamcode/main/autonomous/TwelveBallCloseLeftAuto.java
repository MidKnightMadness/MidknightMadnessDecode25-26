package org.firstinspires.ftc.teamcode.main.autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;

@Autonomous(name = "12 Close Left", group = "Competition")
public class TwelveBallCloseLeftAuto extends TwelveBallCloseRightAuto{
    static{
        startPose = new Pose(180 - 118.7, 130, Math.toRadians(137));
        motifDetectionPose = new Pose(180 - 87, 94, Math.toRadians(80));
        shootPose = new Pose(180 - 87, 94, Math.toRadians(310));
        parkPose = new Pose(180 - 85, 67, Math.toRadians(180));
        openGatePose = new Pose(180 - 136, 76, Math.toRadians(0));
        gateControlPose = new Pose(180 - 114, 76, Math.toRadians(90));
        intakeOnePose = new Pose(180 - 110, 84, Math.toRadians(180));
        intakeTwoPose = new Pose(180 - 110, 60, Math.toRadians(180));
        intakeThreePose = new Pose(180 - 110, 36, Math.toRadians(180));
        intakeDistForward = 14;
        shootSide = ShootSide.LEFT;
    }
}
