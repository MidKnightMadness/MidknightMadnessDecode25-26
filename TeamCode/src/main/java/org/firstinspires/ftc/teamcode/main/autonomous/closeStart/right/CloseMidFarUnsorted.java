package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseGateOpenMidUnsort;

@Autonomous(name = "CR CloseMidFar", group = "BCloseRight")
public class CloseMidFarUnsorted extends CloseGateOpenMidUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
