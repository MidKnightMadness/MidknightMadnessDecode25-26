package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;


@Autonomous(name = "Three Lines Left Back", group = "Competition")
public class ThreeLinesLeft extends ThreeLines {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
