package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCamUnoptimized;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Autonomous(name = "FRB CornerFar CamNonOptim", group = "RightFar")
public class CornerFarCamRight extends CornerFarCamUnoptimized {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}