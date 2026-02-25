package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCamUnoptimized;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Autonomous(name = "FLB CornerFar CamNonOptim", group = "LeftFar")
public class CornerFarCamLeft extends CornerFarCamUnoptimized {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}