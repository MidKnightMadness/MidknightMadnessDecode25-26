package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoClean;

@TeleOp
public class DanielFarAutoLeft extends DanielFarAutoClean {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.LEFT;
    }
}
