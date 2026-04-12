package org.firstinspires.ftc.teamcode.commands.spindexer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;


public class SpindexerGotoPositionSmooth extends CommandBase {
    private final double targetPosition;//position in 0-1
    private final SpindexerNonCR spindexer;
    boolean first;

    double totalTime = 0;
    double previousSetPosition = 0;
    public double dir = 0;

    Timer timer;
    double startValue;
    //total time in sec
    public SpindexerGotoPositionSmooth(SpindexerNonCR spindexer, double position, double totalTime) {
        this.targetPosition = position;
        this.spindexer = spindexer;
        this.totalTime = totalTime;
        addRequirements(this.spindexer);
        timer = new Timer();
    }

    @Override
    public void initialize(){
        previousSetPosition = spindexer.getServo1().getPosition();
        startValue = spindexer.getServo1().getPosition();
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.getTime() / 1000.0;

        spindexer.setDirectPosition(startValue + (targetPosition - startValue) * time/totalTime);
    }

    @Override
    public boolean isFinished() {
        return timer.getTime() / 1000.0 > totalTime;
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted) {//TODO: SEE IF WORKS
            spindexer.setDirectPosition(targetPosition);
        }
    }


}
