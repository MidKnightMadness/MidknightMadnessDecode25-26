package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

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
    SpindexerSpot nearestSpot;
    double timeout_MS;


    Timer timer;
    double startTime = 0;
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
        nearestSpot = spindexer.getNearestEmptyIntakeSpot();


        intake.setDirectPower(power);
        if(nearestSpot != null) {
            spindexer.goToSpot(nearestSpot, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
            swapSpots = true;
        };
        timer = new Timer();
        timer.restart();
        startTime = timer.getTime();
    }


    @Override
    public void execute(){

        intake.setDirectPower(power);
        spindexer.updateBallColors();

        if(swapSpots && nearestSpot != null){
            if(spindexer.isAtSpot(nearestSpot, SpotType.INTAKE)){
                swapSpots = false;
            }
        }

        if(!swapSpots && spindexer.newBallDetected()){
            nearestSpot = spindexer.getNearestEmptyIntakeSpot();
            if(nearestSpot != null){
                spindexer.goToSpot(nearestSpot, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
                swapSpots = true;
            }
        }


    }

    @Override
    public boolean isFinished(){

        if(timer.getTime() - startTime >= timeout_MS){
            intake.setDirectPower(0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
        spindexer.getTurner().set(0);
        swapSpots = false;
    }
}
