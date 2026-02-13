package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarPark;

@Autonomous(name = "Corner Far Park Left Back", group = "Competition")
public class CornerFarParkLeft extends CornerFarPark {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
