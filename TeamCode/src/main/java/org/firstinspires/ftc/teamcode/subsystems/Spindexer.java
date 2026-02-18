package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.color.BallDetector;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.game.MotifEnums;

import org.firstinspires.ftc.teamcode.hardware.RangerMode;
import org.firstinspires.ftc.teamcode.hardware.SwyftRanger;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Configurable
public class Spindexer extends SubsystemBase {

    //Ball sensors(two) facing each other right in the intake before it goes into the spindexer
    //public static double intakeSpinPower = 0.3;
    public static double shootRawPower = 1;

    public static PIDFCoefficients outtakeTurnerCoeff = new PIDFCoefficients(0.00025, 0, 0.6, 0.08);
    public static PIDFCoefficients zeroBallGains = new PIDFCoefficients(0.00015, 0, 0.4, 0.08);
    public static PIDFCoefficients oneBallGains = new PIDFCoefficients(0.0002, 0, 0.6, 0.08);
    public static PIDFCoefficients twoBallGains = new PIDFCoefficients(0.0004, 0, 0.8, 0.08);
    public static PIDFCoefficients intakeTurnerCoeff = new PIDFCoefficients(0.004, 0.002, 0.03, 0.00006);

    //COLOR STUFF
    public BallColor color1 = null;
    public BallColor color2 = null;
//    public BallColor color3 = null;
    int loopNum = 0;
   /// public static PIDFCoefficients turnerCoefficients = new PIDFCoefficients(0.01, 0, 0.001, 0.001);
    // 0 is defined as the position of the shooter
    public static Angle detectRange = Angle.fromDegrees(25); // How far off from the center of the spot that you detect. You don't want to trust measurements that are too off from the center
    public static Angle defaultFinishedThreshold = Angle.fromDegrees(5); // Threshold at which it's finished turning to a spot
    public static Angle finishedThreshold = Angle.fromDegrees(15);//changed from 20
    public static Angle detectThreshold = Angle.fromDegrees(15);
    //0 degrees is facing intake
    //assuming layout at start is initialized as 0 from this position
    //  X X
    //   X
    double distance = 0;
    private static final int NUM_SPOTS = 3;
    boolean useColorSensors;
    CRServoEx2<IncrementalEncoder> turner;
//    CRServoEx2<IncrementalEncoder> turner2;
    public BallDetector[] ballDetectors;
    Angle currentAngle;
    BallColor[] ballColors;
    BallColor[] ballColorsPrev;
    SpindexerSpot[] sequence;
    boolean shootOn = false;
    public PIDFCoefficients getZeroBallCoeff(){
        return zeroBallGains;
    }
    public PIDFCoefficients getOneBallCoeff(){
        return oneBallGains;
    }
    public PIDFCoefficients getTwoBallCoeff(){
        return twoBallGains;
    }

    public boolean outakeSpindexerCoeffOn = false;
    public static double dangerZoneDeg = 15;
    BallColor newBallType = null;

    double overcomeWheelPower = 0.3;
    public SwyftRanger ranger;
    public static double distSensorLowerThreshold = 0;
    public static double distSensorUpperThreshold = 4;
    boolean useDistanceSensor = false;
    IncrementalEncoder turnerEncoder;

    public IncrementalEncoder getTurnerEncoder(){
        return turnerEncoder;
    }
    public static double minDetectDistance = 6;//inch
    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }
    public void setMode(CRServoEx2.RunMode spindexerRunMode) {
        turner.setRunMode(spindexerRunMode);
//        turner2.setRunMode(spindexerRunMode);
    }
    public Spindexer(HardwareMap hardwareMap, boolean useDistanceSensors) {
        this(hardwareMap, useDistanceSensors, null);
    }

    public Spindexer(HardwareMap hardwareMap, boolean useDistanceSensors, BallColor[] ballColors) {
        turnerEncoder = new IncrementalEncoder(
                hardwareMap, ConfigNames.turnerEncoder, 8192, AngleUnit.DEGREES
        ).setReversed(true);
        turner = new CRServoEx2<>(
                hardwareMap, ConfigNames.turner,
                turnerEncoder, CRServoEx2.RunMode.OptimizedPositionalControl
        ).setPIDFTOUse(outtakeTurnerCoeff);//default pid is intake
//        turner2 =  new CRServoEx2<>(
//                hardwareMap, ConfigNames.turner2,
//                turnerEncoder, CRServoEx2.RunMode.OptimizedPositionalControl
//        ).setPIDFTOUse(outtakeTurnerCoeff).setReversed(true);
//        this.useColorSensors = useColorSensors;
        this.useDistanceSensor = useDistanceSensors;


        if(useDistanceSensor){
            ranger = new SwyftRanger(hardwareMap, ConfigNames.intakeDist1, RangerMode.DEG15);
        }

        if(ballColors!= null){
            setBallColors(ballColors);
        }
    }

    @Override
    public void periodic() {
        currentAngle = Angle.fromDegrees(turner.getEncoder().getAngle());

//        if (useDistanceSensor || useColorSensors) updateBalalColors();
        //update color sensor balls manually
//        if(ballColors != null) {
//            ballColorsPrev = ballColors.clone();
//        }
     //   checkHitBottomFlywheel();
    }


    public SpindexerSpot[] getSequence(){
        return sequence;
    }

    public Spindexer initAngle() {
        return initAngle(Angle.fromDegrees(0));
    }
    public double targetSpot;

    // angle is relative to spot 0, so take negative
    public Spindexer initAngle(Angle angle) {
        turner.getEncoder().setAngle(angle.toDegrees());
//        turner2.getEncoder().setAngle(angle.toDegrees());
        currentAngle = angle;
        return this;
    }

    public CRServoEx2<IncrementalEncoder> getTurner() {
        return turner;
    }

//    public CRServoEx2<IncrementalEncoder> getTurner2() {
//        return turner2;
//    }


    public IncrementalEncoder getEncoder() {
        return turner.getEncoder();
    }

    public Angle getCurrentAngle() {
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

    public Spindexer setBallColors(BallColor[] ballColors) {
        this.ballColors = ballColors;
        return this;
    }
    public Spindexer setDefault(){
        this.ballColors[0] = BallColor.NONE;
        this.ballColors[1] = BallColor.NONE;
        this.ballColors[2] = BallColor.NONE;
        return this;
    }

    public BallColor newBallDetected(){
        return newBallType;
    }

    public BallColor getBallColor1(){
        return color1;
    }
    public BallColor getBallColor2(){
        return color2;
    }
    public boolean nearWheel = false;
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
//            if(useColorSensors) {
//                color1 = ballDetectors[0].getColor();
//                color2 = ballDetectors[1].getColor();
//            } else {
//                color1 = BallColor.NONE;
//                color2 = BallColor.NONE;
//            }


//            if (color1 == BallColor.PURPLE || color2 == BallColor.PURPLE) {
//                ballColors[spotIndex] = BallColor.PURPLE;
//            } else if (color1 == BallColor.GREEN || color2 == BallColor.GREEN) {
//                ballColors[spotIndex] = BallColor.GREEN;
//            } else {
//                ballColors[spotIndex] =  BallColor.UNKNOWN;
//            }
            ballColors[spotIndex] = BallColor.UNKNOWN;
            return true;
        }
        return false;
    }

    boolean checkDist() {
        double distance = ranger.getDistance();
        return distance > distSensorLowerThreshold && distance < distSensorUpperThreshold;
    }
    public void updateBallColors() {
        SpindexerSpot spot = getNearestSpot(currentAngle, SpotType.INTAKE);
        if(ballColors == null){
            newBallType = null;
            return;
        }
        if(ballColors[spot.getIndex()] == BallColor.UNKNOWN || !useDistanceSensor){//already has a color
            newBallType = null;
            return;//assume ball stays in position
        }
        if (!(currentAngle.smallestAbsDifferenceDegrees(spot.getSpotAngle(SpotType.INTAKE)).toDegrees()
                < detectThreshold.abs().toDegrees())){
            //don't know with complete certainty which one the ball went into
//            ballColors[spot.getIndex()] = BallColor.NONE;
            newBallType = null;
            return;
        }


//        dist2 = distanceSensor2.getDistance(DistanceUnit.INCH);
        boolean distCheck = checkDist();

        if(distCheck) {
//            if(useColorSensors) {
//                color1 = ballDetectors[0].getColor();
//                color2 = ballDetectors[1].getColor();
//            } else {
//                color1 = BallColor.NONE;
//                color2 = BallColor.NONE;
//            }
//
//
//            if (color1 == BallColor.PURPLE || color2 == BallColor.PURPLE) {
//                ballColors[spot.getIndex()] = BallColor.PURPLE;
//            } else if (color1 == BallColor.GREEN || color2 == BallColor.GREEN) {
//                ballColors[spot.getIndex()] = BallColor.GREEN;
//            } else {
                ballColors[spot.getIndex()] =  BallColor.UNKNOWN;
//            }
        }

    }

    private int computeMomentum(SpindexerSpot[] seq, SpotType spotType, int i) {
        if (i == 0) return getRelativeAngle(seq[i], spotType).sign();
        int diff = (seq[i].getIndex() - seq[i-1].getIndex() + NUM_SPOTS) % NUM_SPOTS;
        if (diff == 0) return 0;
        return (diff <= NUM_SPOTS / 2) ? 1 : -1;
    }

    private SpindexerSpot getNextOuttakeSpot(SpindexerSpot[] seq, int i, int momentum, BallColor ballColor) {
        SpindexerSpot spot;
        if (i == 0) {
            spot = findNearestSpot(currentAngle, SpotType.OUTTAKE, ballColor);
        }
        else {
            int nextIndex = (seq[i - 1].getIndex() + momentum + NUM_SPOTS) % NUM_SPOTS;
            while (ballColors[nextIndex] == BallColor.NONE) {
                nextIndex = (nextIndex + 1) % NUM_SPOTS;
            }
            spot = SpindexerSpot.fromIndex(nextIndex);
        }
        return spot;
    }
    private SpindexerSpot getNextOuttakeSpot(SpindexerSpot[] seq, int i, int momentum) {
        SpindexerSpot spot;
        if (i == 0) {
            spot = getNearestSpot(currentAngle, SpotType.OUTTAKE);
        }
        else {
            int nextIndex = (seq[i - 1].getIndex() + momentum + NUM_SPOTS) % NUM_SPOTS;
            while (ballColors[nextIndex] == BallColor.NONE) {
                nextIndex = (nextIndex + 1) % NUM_SPOTS;
            }
            spot = SpindexerSpot.fromIndex(nextIndex);
        }
        return spot;
    }



    private void checkHitBottomFlywheel(){
        //get nearest spot to outtake
        SpindexerSpot closestSpot = getNearestSpot(currentAngle, SpotType.OUTTAKE);
        if(ballColors[closestSpot.getIndex()] != BallColor.NONE)

        if(!shootOn && closestSpot.getOuttakeAngle().smallestAbsDifferenceDegrees(currentAngle).toDegrees() < dangerZoneDeg){
            if(closestSpot != null) {
                nearWheel = ballColors[closestSpot.getIndex()] != BallColor.NONE;
            }
        }
        double powerDir = turner.power > 0 ? 1 : -1;
        double errorDir = turner.error > 0 ? 1 : -1;

        if(nearWheel) {
            nearWheel = powerDir == errorDir;
        }
//        turner.setMinPowerOvercome(shouldBoost ? overcomeWheelPower * -errorDir : 0);
//        turner2.setMinPowerOvercome(shouldBoost ? overcomeWheelPower * -errorDir: 0);
//
    }

    public void updateShootOn(boolean shootOn){
        this.shootOn = shootOn;
    }
    private SpindexerSpot[] sequenceForMotif(MotifEnums.Motif motif, SpindexerSpot greenSpot) {//outtake
        if(motif.equals(MotifEnums.Motif.NONE)) return null;
        SpindexerSpot[] seq = new SpindexerSpot[NUM_SPOTS];

        int momentum = 0;
        for (int i = 0; i < NUM_SPOTS; i++) {
            SpindexerSpot spot;
            if (i == motif.getGreenPosInd()) {
                spot = greenSpot;
            } else {
                spot = getNextOuttakeSpot(seq, i, momentum, motif.getBallColorFromIndex(i));
            }
            seq[i] = spot;
            momentum = computeMomentum(seq, SpotType.OUTTAKE, i);
        }
        return seq;
    }

    private SpindexerSpot[] sequenceDefault(int totalCount) {
        SpindexerSpot[] seq = new SpindexerSpot[NUM_SPOTS];
        int momentum = 0;

        for (int i = 0; i < totalCount; i++) {
            SpindexerSpot spot = getNextOuttakeSpot(seq, i, momentum);
            seq[i] = spot;
            momentum = computeMomentum(seq, SpotType.OUTTAKE, i);
        }

        return seq;
    }


    public SpindexerSpot[] getOptimalSequence(MotifEnums.Motif motif) {
        SpindexerSpot[] sequence;
        int greenSpot = -1, greenCount = 0, purpleCount = 0, noneCount = 0;
        if(ballColors == null){
            return sequenceDefault(NUM_SPOTS);
        }
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == BallColor.GREEN) {
                greenCount++;
                greenSpot = i;
            }
            else if (ballColors[i] == BallColor.PURPLE) purpleCount++;
            else noneCount++;
        }
        if(noneCount == NUM_SPOTS){
            return sequenceDefault(NUM_SPOTS);
        }

        if (!motif.equals(MotifEnums.Motif.NONE) && ((greenCount == 1 && purpleCount == 2) || (greenCount == 2 && purpleCount == 1))){
            sequence = sequenceForMotif(motif, SpindexerSpot.fromIndex(greenSpot));
        } else {
            sequence = sequenceDefault(greenCount + purpleCount);
        }
        this.sequence = sequence;
        return sequence;
    }


    public SpindexerSpot farthestFromAngle(Angle currentAngle, SpotType spotType) {
        SpindexerSpot farthestSpot = null;
        double maxDiff = -1;

        for (SpindexerSpot spot : SpindexerSpot.values()) {
            double diff = currentAngle.absGap(spot.getSpotAngle(spotType)).toDegrees();
            if (diff > maxDiff) {
                maxDiff = diff;
                farthestSpot = spot;
            }
        }

        return farthestSpot;
    }



    // The current angle of a spot relative to the outtake
    public Angle getRelativeAngle(SpindexerSpot spot, SpotType spotType) {
        return currentAngle.sub(getAbsoluteAngle(spot, spotType));
    }


    public Angle getAbsoluteAngle(SpindexerSpot spot, SpotType spotType) {
        return spot.getSpotAngle(spotType);
    }

    public SpindexerSpot findNearestSpot(Angle query, SpotType spotType, BallColor matchColorOrNull) {
        SpindexerSpot bestSpot = null;
        double smallestGap = 300;
        for (int i = 0; i < SpindexerSpot.values().length; i++) {
            SpindexerSpot spot = SpindexerSpot.fromIndex(i);
            if (matchColorOrNull != null && ballColors[i] != matchColorOrNull) continue;
            double gap = query.absGap(spot.getSpotAngle(spotType)).toDegrees();
            if (gap < smallestGap) {
                smallestGap = gap;
                bestSpot = spot;
            }
        }
        return bestSpot;
    }


    public SpindexerSpot getNearestSpot(Angle query, SpotType spotType) {
        return findNearestSpot(query, spotType, null);
    }

    public SpindexerSpot getNearestEmptyIntakeSpot(){
       return getNearestSpot(currentAngle, SpotType.INTAKE, BallColor.NONE);
    }
    public SpindexerSpot getNearestSpot(Angle query, SpotType spotType, BallColor matchColor) {
        return findNearestSpot(query, spotType, matchColor);
    }

    // Sign of power is direction of spin
    public void spin(double power) {
        turner.setRunMode(CRServoEx2.RunMode.RawPower);
        turner.set(power);
//        turner2.setRunMode(CRServoEx2.RunMode.RawPower);
//        turner2.set(power);
    }

    public boolean isAtAngle(Angle angle) {
        return isAtAngle(angle, finishedThreshold);
    }

    public boolean isAtAngle(Angle angle, Angle finishedThreshold) {
        return currentAngle.smallestAbsDifferenceDegrees(angle).toDegrees()
                < finishedThreshold.abs().toDegrees() && Math.abs(turner.power) < 0.01;
    }

    public boolean isAtSpot(SpindexerSpot spot, SpotType spotType) {
        return currentAngle.smallestAbsDifferenceDegrees(spot.getSpotAngle(spotType)).toDegrees()
                < finishedThreshold.abs().toDegrees() && Math.abs(turner.power) < 0.05;
    }
    public boolean isAtSpotDetection(SpindexerSpot spot, SpotType spotType) {
        return currentAngle.smallestAbsDifferenceDegrees(spot.getSpotAngle(spotType)).toDegrees()
                < detectThreshold.abs().toDegrees();
    }
    public void removeBall(int spot) {
        ballColors[spot] = BallColor.NONE;
    }

    public void goToAngle(Angle angle, CRServoEx2.RunMode runMode) {
//        turner.setRunMode(runMode);
//        turner2.setRunMode(runMode);
        if (runMode == CRServoEx2.RunMode.OptimizedPositionalControl) {
            turner.set(angle.toDegrees());
//            turner2.set(angle.toDegrees());
        } else {
            turner.set(angle.toDegrees());
//            turner2.set(currentAngle.add(angle).sign() * shootRawPower);// Careful signs work out
        }

    }
    public void goToAngleWheelCompensation(Angle angle, CRServoEx2.RunMode runMode) {
        turner.setRunMode(runMode);
        if (runMode == CRServoEx2.RunMode.OptimizedPositionalControl) {
            turner.set(angle.toDegrees(), nearWheel);
        } else {
            turner.set(currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
        }
//        turner2.setRunMode(runMode);
//        if (runMode == CRServoEx2.RunMode.OptimizedPositionalControl) {
//            turner2.set(angle.toDegrees(), nearWheel);
//        } else {
//            turner2.set(currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
//        }
    }
    public void goToAngleOptimized(Angle angle) {
        if (turner.getRunmode() == CRServoEx2.RunMode.OptimizedPositionalControl) {
            turner.set(angle.toDegrees(), nearWheel);
        } else {
            turner.set(currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
        }
//        if (turner2.getRunmode() == CRServoEx2.RunMode.OptimizedPositionalControl) {
//            turner2.set(angle.toDegrees(), nearWheel);
//        } else {
//            turner2.set(currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
//        }
    }

    public void goToSpot(SpindexerSpot spot, SpotType spotType, CRServoEx2.RunMode runMode) {
//        if(spotType == SpotType.INTAKE){
//            turner.setPIDFTOUse(intakeTurnerCoeff);
//        }
//        else{
//            turner.setPIDFTOUse(outtakeTurnerCoeff);
//        }
        goToAngle(spot.getSpotAngle(spotType), runMode);
    }

    public void goToSpotOptimized(SpindexerSpot spot, SpotType spotType){
        goToAngleOptimized(spot.getSpotAngle(spotType));
    }
//    public boolean goToColor(BallColor ballColor, SpotType spotType, CRServoEx2.RunMode runMode, boolean wheelCompensation) {
//        double angleClosest = 400;
//        SpindexerSpot closestSpot = null;
//        for(int i = 0; i < NUM_SPOTS; i++){
//            if(ballColor == ballColors[i]){
//                if((currentAngle.absGap(SpindexerSpot.fromIndex(i).getPreOuttakeAngle()).toDegrees()) < angleClosest){
//                    angleClosest = currentAngle.absGap(SpindexerSpot.fromIndex(i).getPreOuttakeAngle()).toDegrees();
//                    closestSpot = SpindexerSpot.fromIndex(i);
//                }
//            }
//        }
//        if(closestSpot != null){
//            goToSpot(closestSpot, spotType, runMode);
//        }
//
//        if(closestSpot == null){
//            return false;
//        } return true;
//    }

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

    public void setAngleTolerance(Angle angle) {
        finishedThreshold = angle;

    }

    public void setDefaultAngleTolerance() {
        finishedThreshold = defaultFinishedThreshold;
    }

    public void setBallColorIndex(int triggeredSpot, BallColor ballColor) {
        ballColors[triggeredSpot] = ballColor;
    }
}
