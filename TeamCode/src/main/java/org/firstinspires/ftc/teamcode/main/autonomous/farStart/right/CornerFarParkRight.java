package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.CornerFarPark;

@Autonomous(name = "FRB CornerFarPark", group = "Competition")
public class CornerFarParkRight extends CornerFarPark {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}