package org.firstinspires.ftc.teamcode.main.autonomous.farStart.left;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCam;
import org.firstinspires.ftc.teamcode.game.ShootSide;
@Autonomous(name = "Corner Far Cam Left", group = "Competition")
public class CornerFarCamLeft extends CornerFarCam {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.LEFT;
    }
}
