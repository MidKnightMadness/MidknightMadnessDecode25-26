package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.PresetUnsortCornerFarMidSort;

@Autonomous(name = "FRB PresetUnsort CornerFar Sort", group = "ARightFar")
public class PresetUnsortCornerFarSorted extends PresetUnsortCornerFarMidSort {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
