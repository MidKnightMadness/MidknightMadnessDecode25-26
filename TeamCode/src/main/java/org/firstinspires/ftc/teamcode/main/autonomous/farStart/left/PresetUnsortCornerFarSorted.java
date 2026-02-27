package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.PresetUnsortCornerFarMidSort;

@Autonomous(name = "FLB PresetUnsort CornerFar Sort", group = "ALeftFar")
public class PresetUnsortCornerFarSorted extends PresetUnsortCornerFarMidSort {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
