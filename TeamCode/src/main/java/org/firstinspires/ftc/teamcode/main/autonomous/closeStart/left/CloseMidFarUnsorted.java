package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseGateOpenMidUnsort;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseMidFar;

@Autonomous(name = "CL CloseMidFar", group = "BCloseLeft")
public class CloseMidFarUnsorted extends CloseGateOpenMidUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
