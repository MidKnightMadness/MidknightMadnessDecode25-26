package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarCam2;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Autonomous(name = "Corner Far Cam 2 Left Back", group = "Competition")
public class CornerFarCam2Right extends CornerFarCam2 {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}