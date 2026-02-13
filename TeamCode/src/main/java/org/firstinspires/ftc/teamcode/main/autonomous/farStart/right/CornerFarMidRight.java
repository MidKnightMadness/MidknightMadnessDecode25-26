package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarMid;

@Autonomous(name = "Corner Far Mid Right Back", group = "Competition")
public class CornerFarMidRight extends CornerFarMid {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}