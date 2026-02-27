package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseGateMidFarUnsort;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.CloseGateOpenMidUnsort;

@Autonomous(name = "CL GateOpenMidUnsort", group = "BCloseLeft")
public class CloseGateOpenMidUnsorted extends CloseGateOpenMidUnsort {
        @Override
        public ShootSide getShootSide(){
            return ShootSide.LEFT;
        }
}
