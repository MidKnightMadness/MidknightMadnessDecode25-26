package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;


@Autonomous(name = "FLB FarMidClose Sort", group = "Competition")
public class ThreeLinesLeftSort extends ThreeLines {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
