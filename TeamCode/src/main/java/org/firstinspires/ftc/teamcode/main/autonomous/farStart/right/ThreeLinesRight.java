package org.firstinspires.ftc.teamcode.main.autonomous.farStart.right;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.farStart.base.ThreeLines;

@Autonomous(name = "Three Lines Right Back", group = "Competition")
public class ThreeLinesRight extends ThreeLines {

    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}