package org.firstinspires.ftc.teamcode.commands.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.Timer;


//Auto (sorting based)

public class AutoIntakeCommandMotif extends CommandBase {
    private SpindexerNonCR spindexer;
    private Intake intake;
    private double intakePower;

    boolean swapSpots = false;
    Timer timer;
    double startTime;
    public int currNumIndex = 0;
    public int currSpot = 0;
    boolean ballJustDetected;
    public double ballDetectionTime;
    double waitSettle = 0;
    public double time;
    double maxSwapTime1 = 500;
    double maxSwapTime2 = 500;
    boolean atSpot = false;

    boolean ballDetected = false;
    boolean exitTime = false;
    int numBall = 1;
    boolean timeExit;
    HardwareMap hardwareMap;
    BallColor[] ballColors;
    MotifEnums.Motif targetMotif;
    MotifEnums.Motif intakeOrder;
    int[] spotsSeq;
    int stopIntakeNum = -1;
    //when currNumSpot reaches this num, it will stop the intake until its finished rotating to that spot
    //-1 by default meaning doesn't need to stop at any pt
    public AutoIntakeCommandMotif(SpindexerNonCR spindexer, Intake intake, double intakePower, double inBetweenTime, MotifEnums.Motif targetMotif, MotifEnums.Motif intakeOrder, HardwareMap hardwareMap){
        this.spindexer = spindexer;
        this.intake = intake;
        this.intakePower = intakePower;
        this.waitSettle = inBetweenTime;
        this.timeExit = false;
        this.hardwareMap = hardwareMap;
        this.targetMotif = targetMotif;
        this.intakeOrder = intakeOrder;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }
    public AutoIntakeCommandMotif(SpindexerNonCR spindexer, Intake intake, double intakePower, double inBetweenTime, double maxSwapTime1, double maxSwapTime2, HardwareMap hardwareMap,  MotifEnums.Motif targetMotif, MotifEnums.Motif intakeOrder){
        this.spindexer = spindexer;
        this.intake = intake;
        this.intakePower = intakePower;
        this.waitSettle = inBetweenTime;
        this.timeExit = true;
        this.maxSwapTime2 = maxSwapTime2;
        this.maxSwapTime1 = maxSwapTime1;
        this.hardwareMap = hardwareMap;
        this.targetMotif = targetMotif;
        this.intakeOrder = intakeOrder;

        timer = new Timer();
        addRequirements(intake, spindexer);
    }

    @Override
    public void initialize(){
        timer.restart();
        startTime = timer.getTime();
        ballColors = spindexer.getBallColors();
        getSpotSeqFromMotif(targetMotif, intakeOrder);
        currNumIndex = 0;
        currSpot = spotsSeq[0];
        spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(currSpot).getIntakePositionSolo());
    }

    //hardcode motif cases to corerct order to intake, direction, and PGP-PGP exception stop intake case
    public void getSpotSeqFromMotif(MotifEnums.Motif targetMotif, MotifEnums.Motif intakeOrder){
        if(targetMotif == MotifEnums.Motif.PPG){
            if(intakeOrder == MotifEnums.Motif.PPG){
                spotsSeq = new int[]{0, 1, 2};
            } else if(intakeOrder == MotifEnums.Motif.PGP){
                spotsSeq = new int[]{1, 2, 3};
            } else{//GPP
                spotsSeq = new int[]{2, 1, 0};
            }
        } else if (targetMotif == MotifEnums.Motif.GPP) {
            if(intakeOrder == MotifEnums.Motif.PPG){
                spotsSeq = new int[]{3, 2, 1};
            } else if(intakeOrder == MotifEnums.Motif.PGP){
                spotsSeq = new int[]{2, 1, 0};
            } else{//GPP
                spotsSeq = new int[]{1, 2, 3};
            }
        } else{//default motif will be PGP
            if(intakeOrder == MotifEnums.Motif.PPG){
                spotsSeq = new int[]{1, 2, 3};
            } else if(intakeOrder == MotifEnums.Motif.PGP){//annoying case
                spotsSeq = new int[]{2, 3, 1};
                stopIntakeNum = 1;//stops when going to the last position(wraparound)
            } else{//GPP
                spotsSeq = new int[]{3, 2, 1};
            }
        }

    }
    boolean updateStartTime = false;
    double currVolt;
    double atSpotTime = 0;
    boolean intakeOn;

    @Override
    public void execute(){
        if(intakeOn){
            currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
            intake.setDirectPower(intakePower, currVolt);
        } else {
            intake.setDirectPower(0);
        }

        if (!atSpot && spindexer.isAtPosition(SpindexerSpotNonCR.fromIndex(currSpot).getIntakePositionSolo())) {
            atSpot = true;
            intakeOn = true;
            atSpotTime = timer.getTime();
            //if at spot, start detecting for balls and turn intake on(for edge case)
        }

        if(atSpot){
            ballDetected = spindexer.updateBallSpot(currSpot % (SpindexerSpotNonCR.numSpots()));
            if(ballDetected && !updateStartTime) {
                ballDetectionTime = timer.getTime();
                updateStartTime = true;
            }
        }

        time = timer.getTime();
        exitTime = timeExit && (numBall == 1) ? time - atSpotTime >= maxSwapTime1 : timeExit && (numBall == 2) ? time - atSpotTime >= maxSwapTime2 : false;

        if ((ballDetected && ((time - ballDetectionTime >= waitSettle)) || exitTime)) {
            currNumIndex = Math.min(currNumIndex + 1, spotsSeq.length - 1);
            currSpot = spotsSeq[currNumIndex];
            if(stopIntakeNum != -1 && currSpot == stopIntakeNum){
               intakeOn = false;
            }
            spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(currSpot).getIntakePositionSolo());

            atSpot = false;
            ballDetected = false;
            updateStartTime = false;
            numBall++;
        }
    }

    public int getCurrSpot(){
        return currSpot;
    }

    public int getCurrentIndex(){
        return currNumIndex;
    }

    public int[] getSpotsSeq(){
        return spotsSeq;
    }

    @Override
    public boolean isFinished(){
        if(spindexer.allOccuppiedBallColors() || numBall == 4){
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
