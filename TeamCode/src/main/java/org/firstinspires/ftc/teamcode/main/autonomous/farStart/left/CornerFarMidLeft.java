package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarMidSorted;

@Autonomous(name = "FLB CornerFarMidPark", group = "ALeftFar")
public class CornerFarMidLeft extends CornerFarMidSorted {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
