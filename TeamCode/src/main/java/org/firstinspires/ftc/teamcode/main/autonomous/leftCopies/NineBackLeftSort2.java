package org.firstinspires.ftc.teamcode.main.autonomous.leftCopies;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.NineBackRightSort;

@Config
@Configurable
@Autonomous(name = "9 Far Left Sort2" , group = "Competition")
public class NineBackLeftSort2 extends NineBackRightSort {

    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}