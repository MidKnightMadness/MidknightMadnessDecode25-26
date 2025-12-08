package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexerIntakeTimed extends CommandBase {
    Spindexer spindexer;
    Intake intake;
    double intakePower;
    double spinPower;
    double timeDuration;
    double startTime;
    public SpindexerIntakeTimed(Spindexer spindexer, Intake intake, double intakePower, double spinPower, double timeDuration){
        this.spindexer = spindexer;
        this.intake = intake;
        this.intakePower = intakePower;
        this.spinPower = spinPower;
        this.timeDuration = timeDuration;
        addRequirements(spindexer, intake);
    }


    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
        intake.setDirectPower(intakePower);
        spindexer.getTurner().set(spinPower);
    }

    @Override
    public boolean isFinished(){
        if(System.currentTimeMillis() - startTime >= timeDuration){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
        spindexer.getTurner().getServo().setPower(0);
    }
}

