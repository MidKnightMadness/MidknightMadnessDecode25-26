package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.ThreeLines;

@Config
@Configurable
@Disabled
@Autonomous(name = "Three Lines Right Close" , group = "Competition")
public class ThreeLinesRight extends ThreeLines {
    @Override
    public ShootSide getShootSide(){
        return ShootSide.RIGHT;
    }
}
