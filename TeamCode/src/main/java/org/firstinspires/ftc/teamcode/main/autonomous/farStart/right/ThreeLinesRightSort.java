package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;

@Autonomous(name = "FRB FarMidClose Sort", group = "ARightFar")
public class ThreeLinesRightSort extends ThreeLines {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}