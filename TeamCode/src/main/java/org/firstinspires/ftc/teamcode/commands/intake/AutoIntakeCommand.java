package org.firstinspires.ftc.teamcode.commands.intake;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@Config
public class AutoIntakeCommand extends CommandBase {
    private Spindexer spindexer;
    private Intake intake;
    private double power;

    boolean swapSpots = false;
    double timeout_MS;

    public static double[] betweenBallThresholds = new double[]{0, 2000, 4000};

    Timer timer;
    double startTime;
    int currNumBall = 0;
    public AutoIntakeCommand(Spindexer spindexer, Intake intake, double power, double timeOutMS){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.timeout_MS = timeOutMS;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        currNumBall = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE).getIndex();
    }


    @Override
    public void execute(){
        intake.setDirectPower(power);
        spindexer.updateBallColors();
        boolean detectedBall = spindexer.updateProximity();

        //need to have a ball there and be at the spot for it to move on
        SpindexerSpot empty = spindexer.getNearestEmptyIntakeSpot();
        if(empty == null || currNumBall == -1){
            return;
        }
        if(spindexer.isAtSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE) && (detectedBall || spindexer.getBallColors()[currNumBall] != BallColor.NONE)){
            currNumBall  = empty.getIndex();
        }

        //need to call continually
        if(currNumBall != -1) {
            spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }


    }

    @Override
    public boolean isFinished(){
        if(timer.getTime() - startTime >= timeout_MS || spindexer.allOccuppiedBallColors()){
            intake.setDirectPower(0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
        spindexer.getTurner().getServo().setPower(0);
        swapSpots = false;
    }
}
