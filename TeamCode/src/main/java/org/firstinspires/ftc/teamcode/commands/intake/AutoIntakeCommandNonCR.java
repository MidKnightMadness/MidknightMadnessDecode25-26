package org.firstinspires.ftc.teamcode.commands.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;


//teleop (nonsorting based)

public class AutoIntakeCommandNonCR extends CommandBase {
    private SpindexerNonCR spindexer;
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
    public boolean atSpot = false;

    boolean ballDetected = false;
    boolean exitTime = false;
    int numBall = 1;
    public static int addition = 1;
    boolean timeExit;
    HardwareMap hardwareMap;
    BallColor[] ballColors;
    boolean firstBall;
    boolean secondBall;
    boolean thirdBall;
    public SpindexerSpotNonCR startSpot;
    public int dir;
    public AutoIntakeCommandNonCR(SpindexerNonCR spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor, HardwareMap hardwareMap, SpindexerSpotNonCR startSpot, int dir){
        this.spindexer = spindexer;
        this.intake = intake;
        this.power = power;
        this.waitSettle = inBetweenTime;
        this.timeExit = false;
        this.useDistanceSensor = useDistanceSensor;
        this.hardwareMap = hardwareMap;
        this.startSpot = startSpot;
        this.dir = dir;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }
    public AutoIntakeCommandNonCR(SpindexerNonCR spindexer, Intake intake, double power, double inBetweenTime, boolean useDistanceSensor, double maxSwapTime1, double maxSwapTime2, HardwareMap hardwareMap){
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
        ballColors = spindexer.getBallColors();
        this.currNumSpot = this.startSpot.getIndex();

//        firstBall = ballColors[0] != BallColor.NONE;
//        secondBall = ballColors[1] != BallColor.NONE;
//        thirdBall = ballColors[2] != BallColor.NONE;
//
//        if(!firstBall && !secondBall && !thirdBall){
//            currNumSpot = startSpot.getIndex();
//            currRev = startRev;
//        } else if(!firstBall && !thirdBall){
//            currNumSpot = 2;
//            currRev = 0;
//        } else {
//            currNumSpot = 0;
//            currRev = 1;
//        }
        intake.setDirectPower(power);
        spindexer.setDirectPosition(startSpot.getIntakePositionSolo());
    }

    boolean updateStartTime = false;

    boolean exit = false;
    double currVolt;
    @Override
    public void execute(){
//        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        intake.setDirectPower(power, currVolt);

        if (!atSpot && spindexer.isAtPosition(SpindexerSpotNonCR.fromIndex(currNumSpot).getIntakePositionSolo())) {
            atSpot = true;
        }

        if(atSpot){
            ballDetected = spindexer.updateBallSpot(currNumSpot % 3);

            if(ballDetected && !updateStartTime) {
                ballDetectionTime = timer.getTime();
                updateStartTime = true;
            }
        }

//        time = timer.getTime();
//        exitTime = timeExit && (numBall == 1) ? time - ballDetectionTime >= maxSwapTime1 : timeExit && (numBall == 2) ? time - ballDetectionTime >= maxSwapTime2 : false;
//
        if (ballDetected  || exitTime) {
            currNumSpot += dir;
            if(currNumSpot == -1 || currNumSpot == 4){
               exit = true;
               currNumSpot -= dir;
            }
            spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(currNumSpot).getIntakePositionSolo());

            atSpot = false;
            ballDetected = false;
            updateStartTime = false;
            numBall++;
        }
    }

    public double getSpotPosition(){
        return SpindexerSpotNonCR.fromIndex(currNumSpot).getIntakePositionSolo();
    }

    @Override
    public boolean isFinished(){
        if(spindexer.allOccuppiedBallColors() || numBall == 4 || exit){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.setDirectPower(0);
        swapSpots = false;
    }
}
