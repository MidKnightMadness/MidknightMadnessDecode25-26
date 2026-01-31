package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineCloseRightSort;

@Config
@Configurable
@Autonomous(name = "9 Close Left Sort" , group = "Competition")
public class NineCloseLeftSort2 extends NineCloseRightSort {

    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}
