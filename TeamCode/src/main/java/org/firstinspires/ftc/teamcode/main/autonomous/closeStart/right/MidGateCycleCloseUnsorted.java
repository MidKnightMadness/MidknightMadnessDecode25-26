package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.MidGateCycle2Close;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.MidGateCycleClose;

@Autonomous(name = "CR MidGCycle CloseUnsort", group = "BCloseRight")
public class MidGateCycleCloseUnsorted extends MidGateCycleClose {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
