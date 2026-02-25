package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarMidUnsort;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;


@Autonomous(name = "FLB CornerFarMid Unsort", group = "LeftFar")
public class CornerFarMidUnsorted extends CornerFarMidUnsort {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
