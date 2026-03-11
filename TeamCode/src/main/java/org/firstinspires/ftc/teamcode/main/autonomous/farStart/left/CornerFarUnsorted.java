package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarUnsort;

@Autonomous(name = "FLB Corner Far Unsorted", group = "ALeftFar")
public class CornerFarUnsorted extends CornerFarUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
