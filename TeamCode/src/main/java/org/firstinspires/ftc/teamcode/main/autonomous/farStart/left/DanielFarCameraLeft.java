package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoCamera;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoRedone;

@Autonomous
public class DanielFarCameraLeft extends DanielFarAutoCamera {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.LEFT;
    }
}
