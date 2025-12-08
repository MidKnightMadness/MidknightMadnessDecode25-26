package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;

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
    SpindexerSpot nextSpot;
    double timeout_MS;

    public static double[] betweenBallThresholds = new double[]{1000, 2000};

    Timer timer;
    double startTime = 0;
    double inBetweenStart = 0;
    int currSpotIndex = 0;
    public AutoIntakeCommand(Spindexer spindexer, Intake intake, double power, double timeOutMS){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.timeout_MS = timeOutMS;


        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        spindexer.updateBallColors();
        nextSpot = SpindexerSpot.fromIndex(currSpotIndex);


        timer = new Timer();
        timer.restart();
        startTime = timer.getTime();
        inBetweenStart = startTime;
    }


    @Override
    public void execute(){

        intake.setDirectPower(power);
        spindexer.updateBallColors();

        if(swapSpots && nextSpot != null){
            if(spindexer.isAtSpot(nextSpot, SpotType.INTAKE)){
                swapSpots = false;
                currSpotIndex++;
                inBetweenStart = timer.getTime();
            }
        }

        if((!swapSpots && (spindexer.newBallDetected() || spindexer.updateProximity())) || timer.getTime() - inBetweenStart > betweenBallThresholds[currSpotIndex]){
            nextSpot = SpindexerSpot.fromIndex(currSpotIndex);
            if(nextSpot != null){
                spindexer.goToSpot(nextSpot, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
                swapSpots = true;
            }
        }


    }

    @Override
    public boolean isFinished(){
        if(timer.getTime() - startTime >= timeout_MS || currSpotIndex >= 2){
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
