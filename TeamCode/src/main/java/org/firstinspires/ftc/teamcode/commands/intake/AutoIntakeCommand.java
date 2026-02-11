package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;


public class AutoIntakeCommand extends CommandBase {
    private Spindexer spindexer;
    private Intake intake;
    private double power;

    boolean swapSpots = false;
    Timer timer;
    double startTime;
    public int currNumBall = 0;
    boolean ballJustDetected;
    public double ballDetectionTime;
    double waitSettle = 0;
    public double time;
    boolean start = false;
    public AutoIntakeCommand(Spindexer spindexer, Intake intake, double power, double inBetweenTime){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        currNumBall = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE).getIndex();
        intake.setDirectPower(power);
    }

    boolean atSpot = false;

    @Override
    public void execute(){
//        spindexer.updateBallSpot(currNumBall);

        if (!atSpot && spindexer.isAtSpotDetection(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE) && (spindexer.getBallColors()[currNumBall] != BallColor.NONE)) {
            atSpot = true;
            ballDetectionTime = timer.getTime();
        }

        time = timer.getTime();
        if (atSpot && time - ballDetectionTime >= waitSettle) {
            currNumBall = (currNumBall + 1) % 3;
            atSpot = false;
        }


        spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
    }

    @Override
    public boolean isFinished(){
      //  spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);

        if(spindexer.allOccuppiedBallColors()){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
//        spindexer.getTurner().getServo().setPower(0);
//        spindexer.getTurner2().getServo().setPower(0);
        swapSpots = false;
    }
}
