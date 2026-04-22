package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.DanielCloseAutoClean;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.DanielCloseAutoCleanPathing;

@Autonomous
public class DanielClosePathingLeft extends DanielCloseAutoCleanPathing {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.LEFT;
    }
}
