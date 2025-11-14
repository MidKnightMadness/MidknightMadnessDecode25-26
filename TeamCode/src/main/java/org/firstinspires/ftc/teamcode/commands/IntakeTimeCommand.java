package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.List;


@Config
@Configurable
public class IntakeTimeCommand extends CommandBase {
    Intake intake;
    double timeDuration;
    Intake.RunMode runMode = Intake.RunMode.RawPower;
    public static double motorPower = 0.8;
    public static double motorVelocity = 5;
    public static double pGain = 0.01;
    public static double iGain = 0;
    public static double dGain = 0;
    public static double kSGain = 0;
    public static double kVGain = 0;
    public static double kAGain = 0;
    Timer timer;
    double currentTime;
    public IntakeTimeCommand(Intake intake, double timeDuration){
        this.intake = intake;
        this.timeDuration = timeDuration;
        timer = new Timer();

        if(runMode == Intake.RunMode.VelocityControl){
            intake.setPid(pGain, iGain, dGain);
            intake.setFeedforward(kSGain, kVGain, kAGain);
        }
    }

    @Override
    public void initialize(){
        timer.restart();
        intake.resetEncoder();
    }

    @Override
    public void execute(){
        if(runMode == Intake.RunMode.VelocityControl){
            intake.setVelocity(motorVelocity);
        }
        else{
            intake.setDirectPower(motorPower);
        }
    }

    @Override
    public boolean isFinished(){
        boolean finished = timer.getTime() < timeDuration;
        if(finished){
            intake.stopPower();
        }
        return finished;
    }

}
