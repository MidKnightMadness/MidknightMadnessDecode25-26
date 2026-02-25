package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarMidUnsort;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;


@Autonomous(name = "FRB CornerFarMid Unsorted", group = "RightFar")
public class CornerFarMidUnsorted extends CornerFarMidUnsort {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
