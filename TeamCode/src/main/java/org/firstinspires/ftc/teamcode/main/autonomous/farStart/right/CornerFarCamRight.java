package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCam;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Autonomous(name = "Corner Far Cam Right Back", group = "Competition")
public class CornerFarCamRight extends CornerFarCam {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}