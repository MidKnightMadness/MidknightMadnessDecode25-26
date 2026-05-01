package org.firstinspires.ftc.teamcode.commands.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;


//teleop (nonsorting based)

public class AutoIntakeCommandTime extends CommandBase {
    private SpindexerNonCR spindexer;
    private Intake intake;
    private double power;

    boolean swapSpots = false;
    Timer timer;
    double startTime;
    public int currNumSpot = 0;
    public double time;
    public boolean atSpot = false;

    boolean ballDetected = false;
    VoltageSensor voltageSensor;
    BallColor[] ballColors;
    public int startSpot;
    public int dir;

    boolean voltageCompensated;
    double settleTime;
    public AutoIntakeCommandTime(
            SpindexerNonCR spindexer,
            Intake intake,
            double power,
            boolean voltageCompensated,
            VoltageSensor voltageSensor,
            int startSpot,
            int dir,
            double settleTime
    ){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.voltageSensor = voltageSensor;
        this.voltageCompensated = voltageCompensated;
        this.startSpot = startSpot;
        this.dir = dir;
        this.settleTime = settleTime;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }
    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        ballColors = spindexer.getBallColors();
        this.currNumSpot = this.startSpot;
        intake.setDirectPower(power);
        waitingToSettle = true;
        //should be already here
        spindexer.setDirectPosition(SpindexerSpotNonCR.getPositionFromIndex(startSpot, SpotType.INTAKE));
    }


    boolean exit = false;
    boolean waitingToSettle;
    boolean start = false;

    @Override
    public void execute() {
        if(!start){
            timer.restart();
            start = true;
        }

//        if (voltageCompensated) {
//            intake.setDirectPower(power, voltageSensor.getVoltage());
//        }



        if (!atSpot && timer.getTime() >= SpindexerNonCR.SPOT_CHANGE_TIME) {
            waitingToSettle = true;
            timer.restart();
//            intake.setDirectPower(1);
        }

        if (waitingToSettle && timer.getTime() >= settleTime) {
            waitingToSettle = false;
            atSpot = true;
        }

        if (atSpot) {
            int wrappedIndex = ((currNumSpot % 3) + 3) % 3;
            ballDetected = spindexer.updateBallSpot(wrappedIndex);
        }


        if (ballDetected) {
            if (spindexer.allOccuppiedBallColors()) {
                intake.setDirectPower(-1);
                exit = true;
            }
            else {
                currNumSpot += dir;
                spindexer.setPosition(SpindexerSpotNonCR.getPositionFromIndex(currNumSpot, SpotType.INTAKE));
            }

            ballDetected = false;
            atSpot = false;

            // start settle again after movement
            waitingToSettle = false;
            timer.restart();
        }
    }
    public int getFinalSpot(){
        return currNumSpot;
    }

    @Override
    public boolean isFinished(){
        return exit || currNumSpot < 0 || currNumSpot > SpindexerNonCR.TOTAL_SPOTS;
    }

    @Override
    public void end(boolean interrupted){
        swapSpots = false;
    }
}
