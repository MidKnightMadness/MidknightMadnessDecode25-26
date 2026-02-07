package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.MayhemAutoCamBR;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineBackRightLines;

@Config
@Configurable
@Autonomous(name = "Mayhem 9 Bl Cam", group = "Competition")
public class MayhemAutoCamBL extends MayhemAutoCamBR {
    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}