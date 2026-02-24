package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCam;

@Autonomous(name = "FRB CornerFar CamOptim", group = "Competition")
public class CornerFarCamRightOp extends CornerFarCam {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}