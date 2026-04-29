package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoRedone;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoReverted;

@Autonomous
public class DanielFarRevertRight extends DanielFarAutoReverted {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.RIGHT;
    }
}
