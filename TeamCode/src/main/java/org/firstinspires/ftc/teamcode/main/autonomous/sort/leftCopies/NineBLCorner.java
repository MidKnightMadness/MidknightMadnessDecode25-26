package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineBackRightCorner;

@Config
@Configurable
@Autonomous(name = "9 BL Corner, 2 Line", group = "Competition")
public class NineBLCorner extends NineBackRightCorner {
    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}
