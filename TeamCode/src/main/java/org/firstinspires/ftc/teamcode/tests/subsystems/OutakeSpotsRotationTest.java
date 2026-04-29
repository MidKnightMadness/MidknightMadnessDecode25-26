package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.spindexer.OuttakeSpotsRotation;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;


@TeleOp
public class OutakeSpotsRotationTest extends CommandOpMode {

    OuttakeSpotsRotation outakeSpotsRotation;
    SpindexerNonCR spindexer;
    public long totalShootingTime = 700;
    boolean start;
    @Override
    public void initialize() {
        super.reset();
        spindexer = new SpindexerNonCR(hardwareMap, true, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        spindexer.setDirectPosition(SpindexerSpotNonCR.getPositionFromIndex(3, SpotType.INTAKE));
        outakeSpotsRotation = new OuttakeSpotsRotation(spindexer, 3, totalShootingTime / 3, totalShootingTime / 2);

    }

    @Override
    public void run(){
        super.run();
        if(!start){
            schedule(outakeSpotsRotation);
            start = true;
        }

    }
}
