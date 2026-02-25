package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarPark;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarUnsort;

@Autonomous(name = "FRB Corner Far Unsorted", group = "Competition")
public class CornerFarUnsorted extends CornerFarUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
