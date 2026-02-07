package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineBackRightCorner;

@Config
@Configurable
@Disabled
@Autonomous(name = "9 Far Left Sort" , group = "Competition")
public class NineBackLeftSort2 extends NineBackRightCorner {

    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}