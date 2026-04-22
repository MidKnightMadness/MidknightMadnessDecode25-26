package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoClean;

@Autonomous
public class DanielFarAutoRight extends DanielFarAutoClean {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.RIGHT;
    }
}
