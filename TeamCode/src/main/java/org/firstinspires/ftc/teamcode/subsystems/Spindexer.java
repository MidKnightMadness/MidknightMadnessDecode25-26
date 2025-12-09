package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.colors.ColorBuffer;
import org.firstinspires.ftc.teamcode.colors.ColorNormalizer;
import org.firstinspires.ftc.teamcode.colors.ColorSensorBuffer;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ExtraFns;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@Configurable
public class Spindexer extends SubsystemBase {

    //Ball sensors(two) facing each other right in the intake before it goes into the spindexer
    //public static double intakeSpinPower = 0.3;
    public static double shootRawPower = 1;
    public static PIDFCoefficients turnerCoefficients = new PIDFCoefficients(0.003, 0, 0, 0);

    //public static PIDFCoefficients turnerCoefficients = new PIDFCoefficients(0.01, 0, 0.001, 0);
    // 0 is defined as the position of the shooter
    public static Angle detectRange = Angle.fromDegrees(25); // How far off from the center of the spot that you detect. You don't want to trust measurements that are too off from the center
    public static Angle finishedThreshold = Angle.fromDegrees(25); // Threshold at which it's finished turning to a spot
    //0 degrees is facing intake
    //assuming layout at start is initialized as 0 from this position
    //  X X
    //   X
    private static final int NUM_SPOTS = 3;
    boolean useColorSensors;
    CRServoEx2<IncrementalEncoder> turner;
    public BallDetector[] ballDetectors;
    Angle currentAngle;
    BallColor[] ballColors;
    BallColor[] ballColorsPrev;
    SpindexerSpot[] sequence;
    ColorBuffer buffer;
    public static double ballDetectedDistThreshold = 3; // for color sensors

    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }
    public void setMode(CRServoEx2.RunMode spindexerRunMode) {
        turner.setRunMode(spindexerRunMode);
    }
    public Spindexer(HardwareMap hardwareMap, boolean useColorSensors) {
        this(hardwareMap, useColorSensors, null);
    }

    public Spindexer(HardwareMap hardwareMap, boolean useColorSensors, BallColor[] ballColors) {
        IncrementalEncoder turnerEncoder = new IncrementalEncoder(
                hardwareMap, ConfigNames.turnerEncoder, 8192, AngleUnit.DEGREES
        ).setReversed(true);
        turner = new CRServoEx2<>(
                hardwareMap, ConfigNames.turner,
                turnerEncoder, CRServoEx2.RunMode.OptimizedPositionalControl
        ).setPIDF(turnerCoefficients).setReversed(true);
        this.useColorSensors = useColorSensors;
        if (useColorSensors) {
            ballDetectors = new BallDetector[] {
                    new BallDetector(hardwareMap, ConfigNames.intakeColorLeft),
                    new BallDetector(hardwareMap, ConfigNames.intakeColorRight)
            };
        }
        if(ballColors!= null){
            setBallColors(ballColors);
        }
        buffer = new ColorBuffer();
    }

    @Override
    public void periodic() {
        currentAngle = Angle.fromDegrees(turner.getEncoder().getAngle());
//        if (useColorSensors) updateBallColors();
        //update color sensor balls manually
        if(ballColors != null) {
            ballColorsPrev = ballColors.clone();
        }
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
        currentAngle = angle.neg();
        turner.getEncoder().setAngle(-angle.toDegrees());
        return this;
    }

    public CRServoEx2<IncrementalEncoder> getTurner() {
        return turner;
    }

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
            if(ballColor != BallColor.NONE){
                works = false;
            }
        }
        return works;
    }
    public Spindexer setBallColors(BallColor[] ballColors) {
        if(ballColors != null && ballColors.length == 3){
            this.ballColors = ballColors;
        }
        else{
            this.ballColors = new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE};
        }
        return this;
    }

    public boolean newBallDetected(){
        for(int i = 0; i < ballColors.length; i++){
            if(ballColorsPrev[i] == BallColor.NONE && ballColors[i] != BallColor.NONE){
                return true;
            }
        }
        return false;
    }
    public void updateBallColors() {
        SpindexerSpot spot = getNearestSpot(currentAngle, SpotType.INTAKE);
        if(ballColors == null){
            return;
        }
        if(ballColors[spot.getIndex()] != BallColor.NONE){//already has a color
            return;//assume ball stays in position
        }
        if (spot.getSpotAngle(SpotType.INTAKE).absGap(currentAngle).toDegrees() > detectRange.toDegrees()){
            //don't know with complete certainty which one the ball went into
            ballColors[spot.getIndex()] = BallColor.NONE;
            return;
        }


        for(BallDetector sensor: ballDetectors){
            buffer.add(sensor.readColor());
        }

        ballColors[spot.getIndex()] = buffer.getColor();


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

    private SpindexerSpot findNearestSpot(Angle query, SpotType spotType, BallColor matchColorOrNull) {
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
        double minSpotDiff = 400;
        int nearestSpot = 0;
        for(int i = 0; i < ballColors.length; i++){
            Angle spotAngle = SpindexerSpot.fromIndex(i).getIntakeAngle();
            if(ballColors[i] == BallColor.NONE){
                double currSpotDiff = currentAngle.absGap(spotAngle).toDegrees();
                if(currSpotDiff < minSpotDiff){
                    minSpotDiff = currSpotDiff;
                    nearestSpot = i;
                }
            }
        }
        if(minSpotDiff == 400){
            return null;
        }
        return SpindexerSpot.fromIndex(nearestSpot);
    }
    public SpindexerSpot getNearestSpot(Angle query, SpotType spotType, BallColor matchColor) {
        return findNearestSpot(query, spotType, matchColor);
    }

    // Sign of power is direction of spin
    public void spin(double power) {
        turner.setRunMode(CRServoEx2.RunMode.RawPower);
        turner.set(power);
    }

    public boolean isAtAngle(Angle angle) {
        return isAtAngle(angle, finishedThreshold);
    }

    public boolean isAtAngle(Angle angle, Angle finishedThreshold) {
        return currentAngle.sub(angle).abs().toDegrees()
                < finishedThreshold.abs().toDegrees();
    }

    public boolean isAtSpot(SpindexerSpot spot, SpotType spotType) {
        return getRelativeAngle(spot, spotType).abs().toDegrees()
                < finishedThreshold.abs().toDegrees();
    }

    public void removeBall(int spot) {
        ballColors[spot] = BallColor.NONE;
    }

    public void goToAngle(Angle angle, CRServoEx2.RunMode runMode) {
        turner.setRunMode(runMode);
        if (runMode == CRServoEx2.RunMode.OptimizedPositionalControl) {
            turner.set(angle.toDegrees());
        } else {
            turner.set(currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
        }
    }

    public void goToSpot(SpindexerSpot spot, SpotType spotType, CRServoEx2.RunMode runMode) {
        goToAngle(spot.getSpotAngle(spotType), runMode);
    }

    public boolean goToColor(BallColor ballColor, SpotType spotType, CRServoEx2.RunMode runMode) {
        double angleClosest = 400;
        SpindexerSpot closestSpot = null;
        for(int i = 0; i < NUM_SPOTS; i++){
            if(ballColor == ballColors[i]){
                if((currentAngle.absGap(SpindexerSpot.fromIndex(i).getPreOuttakeAngle()).toDegrees()) < angleClosest){
                    angleClosest = currentAngle.absGap(SpindexerSpot.fromIndex(i).getPreOuttakeAngle()).toDegrees();
                    closestSpot = SpindexerSpot.fromIndex(i);
                }
            }
        }
        if(closestSpot != null){
            goToSpot(closestSpot, spotType, runMode);
        }

        if(closestSpot == null){
            return false;
        } return true;
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

    public boolean updateProximity() {
        boolean ball = false;
        for(int i = 0; i < ballDetectors.length; i++){
            double rawDistance = ballDetectors[i].getProximity();
            if(rawDistance < ballDetectedDistThreshold){
                ball = true;
            }
        }
        return ball;
    }
}
