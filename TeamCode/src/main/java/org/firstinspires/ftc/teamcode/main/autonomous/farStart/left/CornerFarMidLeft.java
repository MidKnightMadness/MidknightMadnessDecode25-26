package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarMid;

@Autonomous(name = "Corner Far Mid Left Back", group = "Competition")
public class CornerFarMidLeft extends CornerFarMid {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
