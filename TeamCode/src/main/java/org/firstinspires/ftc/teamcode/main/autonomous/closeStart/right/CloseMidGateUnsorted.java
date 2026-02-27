package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseMidGateUnsort;


@Autonomous(name = "CR CloseMidGateUnsort", group = "BCloseRight")
public class CloseMidGateUnsorted extends CloseMidGateUnsort {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
