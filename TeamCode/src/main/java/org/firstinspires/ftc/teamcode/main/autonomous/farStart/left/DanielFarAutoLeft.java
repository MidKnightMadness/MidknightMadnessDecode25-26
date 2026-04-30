package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.DanielFarAutoRedone;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.right.DanielFarAutoRight;

//@Autonomous
public class DanielFarAutoLeft extends DanielFarAutoRight {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.LEFT;
    }
}
