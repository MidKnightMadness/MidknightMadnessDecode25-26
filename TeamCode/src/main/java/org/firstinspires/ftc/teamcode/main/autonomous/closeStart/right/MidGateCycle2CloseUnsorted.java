package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.MidGateCycle2Close;

@Autonomous(name = "CR MidGCycle2 CloseUnsort", group = "BCloseRight")
public class MidGateCycle2CloseUnsorted extends MidGateCycle2Close {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
