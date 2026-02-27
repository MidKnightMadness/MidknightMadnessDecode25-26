package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseGateMidFarUnsort;

@Autonomous(name = "CL GateMidFarUnsort", group = "CloseLeft")
public class CloseGateMidFarUnsorted extends CloseGateMidFarUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
