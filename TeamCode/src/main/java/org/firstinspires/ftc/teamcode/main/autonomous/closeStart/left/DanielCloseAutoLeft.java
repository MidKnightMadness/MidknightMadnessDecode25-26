package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.left;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.DanielCloseAuto;

@TeleOp
public class DanielCloseAutoLeft extends DanielCloseAuto {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.LEFT;
    }
}
