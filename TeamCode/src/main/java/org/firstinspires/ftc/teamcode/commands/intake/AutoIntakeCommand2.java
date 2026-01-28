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

public class AutoIntakeCommand2 extends CommandBase {
    private Spindexer spindexer;
    private Intake intake;
    private double power;

    boolean swapSpots = false;
    Timer timer;
    double startTime;
    public int currNumSpot = 0;
    boolean ballJustDetected;
    public double ballDetectionTime;
    double waitSettle = 0;
    public double time;
    boolean useDistanceSensor = true;
    double maxSwapTime1 = 1000;
    double maxSwapTime2 = 1000;
    boolean atSpot = false;

    boolean ballDetected = false;
    boolean exitTime = false;
    int numBall = 1;
    boolean timeExit;
    public AutoIntakeCommand2(Spindexer spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;
        this.timeExit = false;
        this.useDistanceSensor = useDistanceSensor;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }
    public AutoIntakeCommand2(Spindexer spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor, double maxSwapTime1, double maxSwapTime2){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;
        this.useDistanceSensor = useDistanceSensor;
        this.timeExit = true;
        this.maxSwapTime2 = maxSwapTime2;
        this.maxSwapTime1 = maxSwapTime1;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        intake.setDirectPower(power);
        currNumSpot = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE).getIndex();

    }


    @Override
    public void execute(){
        ballDetected = spindexer.updateBallSpot(currNumSpot);

        if (!atSpot && spindexer.isAtSpotDetection(SpindexerSpot.fromIndex(currNumSpot), SpotType.INTAKE) && (spindexer.getBallColors()[currNumSpot] != BallColor.NONE)) {
            atSpot = true;
            ballDetectionTime = timer.getTime();
        }

        time = timer.getTime();
        exitTime = timeExit && (numBall == 1) ? time - ballDetectionTime >= maxSwapTime1 : timeExit && (numBall == 2) ? time - ballDetectionTime >= maxSwapTime2 : false;

        if (atSpot && ((time - ballDetectionTime >= waitSettle && ballDetected)) || exitTime) {
            currNumSpot = (currNumSpot + 1) % 3;
            atSpot = false;
            numBall++;
        }


        spindexer.goToSpot(SpindexerSpot.fromIndex(currNumSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
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
