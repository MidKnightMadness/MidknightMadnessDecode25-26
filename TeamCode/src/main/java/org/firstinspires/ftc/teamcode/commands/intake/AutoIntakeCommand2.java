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
public class AutoIntakeCommand2 extends CommandBase {
    private Spindexer spindexer;
    private Intake intake;
    private double power;

    boolean swapSpots = false;
    double timeout_MS;
    Timer timer;
    double startTime;
    public int currNumBall = 0;
    boolean ballJustDetected;
    public double ballDetectionTime;
    double waitSettle = 0;
    public double time;
    boolean start = false;
    public AutoIntakeCommand2(Spindexer spindexer, Intake intake, double power, double timeOutMS, double inBetweenTime){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.timeout_MS = timeOutMS;
        waitSettle = inBetweenTime;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
    }


    boolean atSpot = false;
    @Override
    public void execute(){
        spindexer.updateBallSpot(currNumBall);
        intake.setDirectPower(power);

        time = timer.getTime();

        //need to have a ball there and be at the spot for it to move on

        boolean spotCheck =  spindexer.isAtSpotDetection(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE);
        boolean colorCheck = spindexer.getBallColors()[currNumBall] != BallColor.NONE;
        if (!atSpot && spotCheck && colorCheck) {
            atSpot = true;
            ballDetectionTime = timer.getTime();
        }




        time = timer.getTime();
        if(atSpot){
            intake.setDirectPower(1.0);
        } else{
            intake.setDirectPower(0);
        }
        if (atSpot && time - ballDetectionTime >= waitSettle) {
            currNumBall = (currNumBall + 1) % 3;
            atSpot = false;
        } else{
            intake.setDirectPower(0);
        }


        spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);




        //need to call continually
        if(currNumBall != -1) {
            spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }

    }

    @Override
    public boolean isFinished(){
//        if(currNumBall != -1) {
//            spindexer.goToSpot(SpindexerSpot.fromIndex(currNumBall), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
//        }

        if(timer.getTime() - startTime >= timeout_MS || spindexer.allOccuppiedBallColors()){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
        spindexer.getTurner().getServo().setPower(0);
        spindexer.getTurner2().getServo().setPower(0);
        swapSpots = false;
    }
}
