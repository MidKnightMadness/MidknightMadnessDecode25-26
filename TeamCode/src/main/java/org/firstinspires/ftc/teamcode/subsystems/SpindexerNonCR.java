package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.hardware.GobildaDistance;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;

import org.firstinspires.ftc.teamcode.hardware.RangerMode;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

import java.util.ArrayList;

@Configurable
public class SpindexerNonCR extends SubsystemBase {
    public double startOutakePosition = 1;
    public double endOutakePosition = 0;
    public static double SPINDEXER_OFFSET_DEGREES = 90d;
    public double startTeleOpIntakePosition = SpindexerSpotNonCR.fromIndex(1).getIntakePositions().get(0);
    public static int totalDegrees = 791;//817 extended gr?
    public static AngleNonCR defaultFinishedThreshold = AngleNonCR.fromDegrees(5); // Threshold at which it's finished turning to a spot
    public static AngleNonCR finishedThreshold = AngleNonCR.fromDegrees(15);//TODO: Change to 15 for auto?
    public static AngleNonCR strictFinished = AngleNonCR.fromDegrees(10);
    public static final int NUM_SPOTS = 3;//for one rotation
    public static int TOTAL_SPOTS = Math.floorDiv(totalDegrees, 120);
    ServoImplEx turner;
    ServoImplEx turner2;
    AngleNonCR currentAngle;
    BallColor[] ballColors;
    boolean shootOn = false;
    BallColor newBallType = null;
    boolean useDistanceSensor;
    public GobildaDistance distanceSensor1;
    public GobildaDistance distanceSensor2;
    //0 - 4 for swyft distance sensor
    public static double distSensorLowerThreshold = 0.5;
    public static double distSensorUpperThreshold = 2.5;
    double cachingTolerance = 0.01;
    public final double DEGREES_PER_SECOND = 120.0 / SPOT_CHANGE_TIME;//deg per ms
    public static double SPOT_CHANGE_TIME = 150;

    public static double STRICT_SPOT_TIME = 120;


    double currentTurnerPosition = 0;
    public SpindexerNonCR(HardwareMap hardwareMap, boolean useDistanceSensors, BallColor[] ballColors) {
        turner = hardwareMap.get(ServoImplEx.class, ConfigNames.turner);
        turner.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turner.setDirection(Servo.Direction.REVERSE);

        turner2 = hardwareMap.get(ServoImplEx.class, ConfigNames.turner2);
        turner2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turner2.setDirection(Servo.Direction.REVERSE);


        this.useDistanceSensor = useDistanceSensors;


        if(useDistanceSensor){
            distanceSensor1 = new GobildaDistance(hardwareMap, ConfigNames.intakeDist1, RangerMode.DEG15);
            distanceSensor2 = new GobildaDistance(hardwareMap, ConfigNames.intakeDist2, RangerMode.DEG15);
        }

        if(ballColors!= null){
            setBallColors(ballColors);
        }
    }

    public Servo getServo1(){
        return turner;
    }

    public Servo getServo2(){
        return turner2;
    }



    public ServoImplEx getServoImplEx1(){
        return turner;
    }

    public ServoImplEx getServoImplEx2(){
        return turner2;
    }


    public void setTeleOpStartIntake(){
        setPosition(startTeleOpIntakePosition);
    }
    public void setStartOuttakePosition(){
        setPosition(startOutakePosition);
    }

    public void setEndOuttakePosition(){
        setPosition(endOutakePosition);
    }

    public void setPosition(double position) {
        turner.setPosition(position);
        turner2.setPosition(position);
    }

    public void setDirectPosition(double position){
        turner.setPosition(position);
        turner2.setPosition(position);
    }


    public double getTimeForRotation(double degrees) {
        return Math.abs(degrees) / DEGREES_PER_SECOND;
    }



    public double getCurrentSpindexerPosition(){
        return currentTurnerPosition;
    }
    @Override
    public void periodic() {
        currentTurnerPosition = turner.getPosition();
        currentAngle = AngleNonCR.fromDegrees(currentTurnerPosition / totalDegrees);
    }


    public SpindexerNonCR initAngle() {
        return initAngle(AngleNonCR.fromDegrees(0));
    }

    public double targetSpot;

    // angle is relative to spot 0, so take negative
    public SpindexerNonCR initAngle(AngleNonCR angle) {
        currentAngle = angle;
        return this;
    }


    public AngleNonCR getCurrentAngle() {
        return currentAngle;
    }

    public BallColor[] getBallColors() {
        return ballColors;
    }

    public boolean allOccuppiedBallColors(){
        boolean works = true;
        for(BallColor ballColor : ballColors){
            if(ballColor == BallColor.NONE){
                works = false;
                break;
            }
        }
        return works;
    }
    public SpindexerNonCR setBallColors(BallColor[] ballColors) {
        this.ballColors = ballColors;
        return this;
    }
    public SpindexerNonCR setDefault(){
        this.ballColors[0] = BallColor.NONE;
        this.ballColors[1] = BallColor.NONE;
        this.ballColors[2] = BallColor.NONE;
        return this;
    }

    public BallColor newBallDetected(){
        return newBallType;
    }

    public boolean distCheck = false;

    public boolean updateBallSpot(int spotIndex){
        if(ballColors == null || !useDistanceSensor){
            return false;
        }
        if(ballColors[spotIndex] == BallColor.UNKNOWN){//already has a color
            return true;//assume ball stays in position
        }

        distCheck = checkDist();

        if(distCheck) {
            ballColors[spotIndex] = BallColor.UNKNOWN;
            return true;
        }
        return false;
    }

    double distance1;
    double distance2;
    boolean dist1Check;
    boolean dist2Check;

    public double getDistance1(){
        return distance1;
    }
    public double getDistance2(){
        return distance2;
    }
    public boolean getDist1Check(){
        return dist1Check;
    }
    public boolean getDist2Check(){
        return dist2Check;
    }


    boolean checkDist() {
        distance1 = distanceSensor1.getDistance();
        distance2 = distanceSensor2.getDistance();
        dist1Check = distance1 > distSensorLowerThreshold && distance1 < distSensorUpperThreshold;
        dist2Check = distance2 > distSensorLowerThreshold && distance2 < distSensorUpperThreshold;
        return dist1Check || dist2Check;
    }

    public void updateShootOn(boolean shootOn){
        this.shootOn = shootOn;
    }
//    private SpindexerSpot[] sequenceForMotif(MotifEnums.Motif motif, SpindexerSpot greenSpot) {//outtake
//        if(motif.equals(MotifEnums.Motif.NONE)) return null;
//        SpindexerSpot[] seq = new SpindexerSpot[NUM_SPOTS];
//
//        int momentum = 0;
//        for (int i = 0; i < NUM_SPOTS; i++) {
//            SpindexerSpot spot;
//            if (i == motif.getGreenPosInd()) {
//                spot = greenSpot;
//            } else {
//                spot = getNextOuttakeSpot(seq, i, momentum, motif.getBallColorFromIndex(i));
//            }
//            seq[i] = spot;
//            momentum = computeMomentum(seq, SpotType.OUTTAKE, i);
//        }
//        return seq;
//    }

//    private SpindexerSpot[] sequenceDefault(int totalCount) {
//        SpindexerSpot[] seq = new SpindexerSpot[NUM_SPOTS];
//        int momentum = 0;
//
//        for (int i = 0; i < totalCount; i++) {
//            SpindexerSpot spot = getNextOuttakeSpot(seq, i, momentum);
//            seq[i] = spot;
//            momentum = computeMomentum(seq, SpotType.OUTTAKE, i);
//        }
//
//        return seq;
//    }
//
//
//    public SpindexerSpot[] getOptimalSequence(MotifEnums.Motif motif) {
//        SpindexerSpot[] sequence;
//        int greenSpot = -1, greenCount = 0, purpleCount = 0, noneCount = 0;
//        if(ballColors == null){
//            return sequenceDefault(NUM_SPOTS);
//        }
//        for (int i = 0; i < ballColors.length; i++) {
//            if (ballColors[i] == BallColor.GREEN) {
//                greenCount++;
//                greenSpot = i;`
//            }
//            else if (ballColors[i] == BallColor.PURPLE) purpleCount++;
//            else noneCount++;
//        }
//        if(noneCount == NUM_SPOTS){
//            return sequenceDefault(NUM_SPOTS);
//        }
//
//        if (!motif.equals(MotifEnums.Motif.NONE) && ((greenCount == 1 && purpleCount == 2) || (greenCount == 2 && purpleCount == 1))){
//            sequence = sequenceForMotif(motif, SpindexerSpot.fromIndex(greenSpot));
//        } else {
//            sequence = sequenceDefault(greenCount + purpleCount);
//        }
//        this.sequence = sequence;
//        return sequence;
//    }



    // The current angle of a spot relative to the outtake
    public AngleNonCR getRelativeAngle(SpindexerSpot spot, SpotType spotType, int num) {
        return currentAngle.diff(getAbsoluteAngle(spot, spotType, num));
    }


    public AngleNonCR getAbsoluteAngle(SpindexerSpot spot, SpotType spotType, int num) {
        return AngleNonCR.fromDegrees(spot.getSpotAngle(spotType).getValue()  * totalDegrees);
    }

//    public SpindexerSpotNonCR findNearestSpot(AngleNonCR query, SpotType spotType, BallColor matchColorOrNull) {
//        SpindexerSpotNonCR bestSpot = null;
//        double smallestGap = 300;
//        for (int i = 0; i < SpindexerSpotNonCR.values().length; i++) {
//            SpindexerSpotNonCR spot = SpindexerSpotNonCR.fromIndex(i);
//            if (matchColorOrNull != null && ballColors[i] != matchColorOrNull) continue;
//            double gap = query.absGap(spot.getSpotAngle(spotType)).toDegrees();
//            if (gap < smallestGap) {
//                smallestGap = gap;
//                bestSpot = spot;
//            }
//        }
//        return bestSpot;
//    }


//    public SpindexerSpotNonCR getNearestSpot(AngleNonCR query, SpotType spotType) {
//        return findNearestSpot(query, spotType, null);
//    }

    //    public SpindexerSpotNonCR getNearestEmptyIntakeSpot(){
//        return getNearestSpot(currentAngle, SpotType.INTAKE, BallColor.NONE);
//    }
    public double getNearestIntakePosition(SpotType spotType) {
        double smallestGap = 1;
        double bestValue = 0;
        for (int i = 0; i < SpindexerSpotNonCR.values().length; i++) {
            SpindexerSpotNonCR spot = SpindexerSpotNonCR.fromIndex(i);
            ArrayList<Double> spotPositions = spot.getSpotPosition(spotType);
            for(double j : spotPositions){
                if(j != -1){
                    double gap = Math.abs(j - currentTurnerPosition);
                    if (gap < smallestGap) {
                        smallestGap = gap;
                        bestValue = j;
                    }
                }
            }
        }
        return bestValue;
    }

    public boolean isAtPosition(double position){
        double angle = position * totalDegrees;
        return isAtAngle(AngleNonCR.fromDegrees(angle), finishedThreshold);
    }
    public boolean isAtPositionStrict(double position){
        double angle = position * totalDegrees;
        return isAtAngleStrict(AngleNonCR.fromDegrees(angle));
    }
    public boolean isAtAngle(AngleNonCR angle, AngleNonCR finishedThreshold) {
        return angle.diff(currentAngle).toDegrees() < finishedThreshold.abs().toDegrees();
    }

    public boolean isAtAngleStrict(AngleNonCR angle){
        return angle.diff(currentAngle).toDegrees() < strictFinished.abs().toDegrees();
    }

    public boolean isAtSpot(SpindexerSpotNonCR spot, SpotType spotType, int spotNum) {
        return Math.abs(spot.getSpotPosition(spotType).get(spotNum) - currentAngle.getValue()) <= finishedThreshold.getValue();
    }
    public void removeBall(int spot) {
        ballColors[spot] = BallColor.NONE;
    }

    public int getSpotOptimal(int swapNumber, int currentSpot){
        //swapNumber either -1 or 1 positive spot increase, negative spot decrease
        //determines optimal spot to set to for sorting
        //bounds: 3<= X <= #Spots
        if(swapNumber == -1){
            currentSpot-= 1;
        } else if(swapNumber == 1){
            currentSpot+=1;
        }

        if(currentSpot < 0){
            currentSpot+=3;
        }
        if(currentSpot > TOTAL_SPOTS){
            currentSpot-=3;
        }

        return currentSpot;
    }


    public int getBallCount() {
        if(ballColors == null){
            return 0;
        }
        int ballsColored = 0;
        for(int i = 0; i < ballColors.length; i++){
            if(ballColors[i] != BallColor.NONE){
                ballsColored++;
            }
        }
        return ballsColored;
    }

    public void setAngleTolerance(AngleNonCR angle) {
        finishedThreshold = angle;
    }

    public void setDefaultAngleTolerance() {
        finishedThreshold = defaultFinishedThreshold;
    }

    public void setBallColorIndex(int triggeredSpot, BallColor ballColor) {
        ballColors[triggeredSpot] = ballColor;
    }
}
