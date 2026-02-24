package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCam;

@Autonomous(name = "FLB CornerFar CamOptim", group = "Competition")
public class CornerFarCamLeftOp extends CornerFarCam {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
