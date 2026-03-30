package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.right;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base.DanielCloseAuto;

@TeleOp
public class DanielCloseAutoRight extends DanielCloseAuto {
    @Override
    public ShootSide getShootSide() {
        return ShootSide.RIGHT;
    }
}