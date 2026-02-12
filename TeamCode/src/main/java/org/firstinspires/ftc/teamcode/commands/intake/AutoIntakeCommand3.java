package org.firstinspires.ftc.teamcode.commands.intake;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;


@Config
@Configurable
public class AutoIntakeCommand3 extends CommandBase {
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
    double maxSwapTime1 = 500;
    double maxSwapTime2 = 500;
    boolean atSpot = false;

    boolean ballDetected = false;
    boolean exitTime = false;
    int numBall = 1;
    public static int addition = 1;
    boolean timeExit;
    HardwareMap hardwareMap;
    public AutoIntakeCommand3(Spindexer spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor, HardwareMap hardwareMap){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;
        this.timeExit = false;
        this.useDistanceSensor = useDistanceSensor;
        this.hardwareMap = hardwareMap;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }
    public AutoIntakeCommand3(Spindexer spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor, double maxSwapTime1, double maxSwapTime2, HardwareMap hardwareMap){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;
        this.useDistanceSensor = useDistanceSensor;
        this.timeExit = true;
        this.maxSwapTime2 = maxSwapTime2;
        this.maxSwapTime1 = maxSwapTime1;
        this.hardwareMap = hardwareMap;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        currNumSpot = 0;
    }

    boolean updateStartTime = false;

    double currVolt;
    @Override
    public void execute(){
//        ballDetected = spindexer.updateBallSpot(currNumSpot);


        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
        intake.setDirectPower(power, currVolt);

        if (!atSpot && spindexer.isAtSpotDetection(SpindexerSpot.fromIndex(currNumSpot), SpotType.INTAKE)) {
            atSpot = true;
        }

        if(atSpot){
            ballDetected = spindexer.updateBallSpot(currNumSpot);
            if(ballDetected && !updateStartTime) {
                ballDetectionTime = timer.getTime();
                updateStartTime = true;
            }
        }

        time = timer.getTime();
        exitTime = timeExit && (numBall == 1) ? time - ballDetectionTime >= maxSwapTime1 : timeExit && (numBall == 2) ? time - ballDetectionTime >= maxSwapTime2 : false;

        if (ballDetected && ((time - ballDetectionTime >= waitSettle)) || exitTime) {
            currNumSpot = (currNumSpot - 2);
//        if (((time - ballDetectionTime >= waitSettle)) || exitTime) {

            if(currNumSpot < 0){
                currNumSpot += 3;
            }
            atSpot = false;
            ballDetected = false;
            updateStartTime = false;
            numBall++;
        }


        spindexer.goToSpot(SpindexerSpot.fromIndex(currNumSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
    }

    public int getSpotCurrent(){
        return currNumSpot;
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
