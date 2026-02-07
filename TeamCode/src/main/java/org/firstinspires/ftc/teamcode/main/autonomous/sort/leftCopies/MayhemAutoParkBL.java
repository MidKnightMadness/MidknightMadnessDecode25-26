package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineBackRightCorner;
import org.firstinspires.ftc.teamcode.main.autonomous.sort.right.NineBackRightLines;

@Config
@Configurable
@Autonomous(name = "Mayhem 9 BL Park", group = "Competition")
public class MayhemAutoParkBL extends NineBackRightLines {
    ShootSide shootSide = ShootSide.LEFT;
    @Override
    public ShootSide getShootSide(){
        return shootSide;
    }
}
