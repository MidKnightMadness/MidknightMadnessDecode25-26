package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ExtraFns;

import java.util.HashMap;
import java.util.Map;

@Configurable
public class Spindexer extends SubsystemBase {
    public static class BallSensor {
        BallDetector sensor;
        Angle angle;
        int previousSpot;
        Map<BallColor, Double> votes;

        BallSensor(BallDetector sensor, Angle angle) {
            this.sensor = sensor;
            this.angle = angle;
            this.previousSpot = -1; // -1 for none
            resetVotes();
        }

        BallColor aggregateVotes() {
            return ExtraFns.argmax(votes);
        }

        void addVote(BallColor color, double weight) {
            votes.merge(color, weight, Double::sum);
        }

        void resetVotes() {
            votes = new HashMap<>();
            votes.put(BallColor.GREEN, 0.0);
            votes.put(BallColor.PURPLE, 0.0);
            votes.put(BallColor.NONE, 0.0);
        }
    }

    public static double intakeSpinPower = 0.3;
    public static double shootRawPower = 1;
    public static PIDFCoefficients turnerCoefficients = new PIDFCoefficients(0.005, 0, 0, 0);

    // 0 is defined as the position of the shooter
    public Angle spotZeroReading = Angle.fromDegrees(0); // Raw encoder reading when spot 0 is aligned with shooter
    public static Angle detectRange = Angle.fromDegrees(40); // How far off from the center of the spot that you detect. You don't want to trust measurements that are too off from the center
    public static Angle inColorSensorAngle = Angle.fromDegrees(180);
    public static Angle outColorSensorAngle = Angle.fromDegrees(0);
    public static Angle finishedThreshold = Angle.fromDegrees(5); // Threshold at which it's finished turning to a spot

    private static final int NUM_SPOTS = 3;

    public Angle test = new Angle(0, AngleUnit.DEGREES);

    boolean useColorSensors;
    CRServoEx2<IncrementalEncoder> turner;
    BallSensor[] ballSensors;
    Angle currentAngle;
    BallColor[] ballColors;

    public Spindexer(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    public Spindexer(HardwareMap hardwareMap, boolean useColorSensors) {
        IncrementalEncoder turnerEncoder = new IncrementalEncoder(
                hardwareMap, "turnerEncoder", 8192, AngleUnit.DEGREES
        );
        turner = new CRServoEx2<>(
                hardwareMap, ConfigNames.turner,
                turnerEncoder, CRServoEx2.RunMode.RawPower
        ).setPIDF(turnerCoefficients).setReversed(true);
        this.useColorSensors = useColorSensors;
        if (useColorSensors) {
            ballSensors = new BallSensor[] {
                    new BallSensor(
                            new BallDetector(hardwareMap, ConfigNames.inColorSensor),
                            inColorSensorAngle
                    ),
                    new BallSensor(
                            new BallDetector(hardwareMap, ConfigNames.outColorSensor),
                            outColorSensorAngle
                    )
            };
        }
        ballColors = new BallColor[] { BallColor.NONE, BallColor.NONE, BallColor.NONE };
    }

    @Override
    public void periodic() {
        currentAngle = Angle.fromDegrees(
                turner.getEncoder().getAngle() - spotZeroReading.toDegrees()
        );
        if (useColorSensors) updateBallColors();
    }

    public Spindexer init() {
        turner.setEncoder(turner.getEncoder().zero());
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

    public void updateBallColors() {
        for (BallSensor ballSensor: ballSensors) {
            int spot = getNearestSpotIndex(ballSensor.angle);
            double gap = getRelativeAngle(spot).absGap(ballSensor.angle).toDegrees();
            double weight = Math.max(0, 1 - gap / detectRange.toDegrees());
            if (gap > detectRange.toDegrees()) spot = -1;

            int previousSpot = ballSensor.previousSpot;
            ballSensor.previousSpot = spot;

            if (spot == previousSpot) { // If in the middle of detecting
                if (spot == -1) continue;
                ballSensor.addVote(ballSensor.sensor.readColor(), weight);
            } else { // If spot transition
                if (previousSpot == -1) continue; // Only consider transitions from spots
                ballColors[previousSpot] = ballSensor.aggregateVotes();
                ballSensor.resetVotes();
            }
        }
    }

    private int computeMomentum(int[] seq, int i, int nextSpot) {
        if (i == 0) return getRelativeAngle(nextSpot).sign();
        int diff = (nextSpot - seq[i-1] + NUM_SPOTS) % NUM_SPOTS;
        if (diff == 0) return 0;
        return (diff <= NUM_SPOTS / 2) ? 1 : -1;
    }

    private int[] sequenceForMotif(MotifEnums.Motif motif, int greenSpot) {
        int greenOrder;
        int[] seq = new int[NUM_SPOTS];
        if (motif.equals(MotifEnums.Motif.GPP)) greenOrder = 0;
        else if (motif.equals(MotifEnums.Motif.PGP)) greenOrder = 1;
        else greenOrder = 2;

        int momentum = 0;
        for (int i = 0; i < NUM_SPOTS; i++) {
            int spot;
            if (i == greenOrder) {
                spot = greenSpot;
            } else {
                spot = getNextSpot(seq, i, momentum);
            }
            seq[i] = spot;
            momentum = computeMomentum(seq, i, spot);
        }
        return seq;
    }

    private int getNextSpot(int[] seq, int i, int momentum) {
        int spot;
        if (i == 0) {
            spot = getNearestSpotIndex(Angle.fromDegrees(0));
        } else {
            spot = (seq[i - 1] + momentum) % NUM_SPOTS;
            while (ballColors[spot] == BallColor.NONE) {
                spot = (spot + 1) % NUM_SPOTS;
            }
        }
        return spot;
    }

    private int[] sequenceDefault(int totalCount) {
        int[] seq = new int[NUM_SPOTS];
        int momentum = 0;

        for (int i = 0; i < totalCount; i++) {
            int spot = getNextSpot(seq, i, momentum);
            seq[i] = spot;
            momentum = computeMomentum(seq, i, spot);
        }

        return seq;
    }

    public int[] getOptimalSequence(MotifEnums.Motif motif) {
        int[] sequence;
        int greenSpot = -1, greenCount = 0, purpleCount = 0;
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == BallColor.GREEN) {
                greenCount++;
                greenSpot = i;
            }
            else if (ballColors[i] == BallColor.PURPLE) purpleCount++;
        }

        if (!motif.equals(MotifEnums.Motif.NONE) && greenCount == 1 && purpleCount == 2) {
            sequence = sequenceForMotif(motif, greenSpot);
        } else {
            sequence = sequenceDefault(greenCount + purpleCount);
        }
        return sequence;
    }

    // The current angle of a spot relative to the outtake
    public Angle getRelativeAngle(int spot) {
        return currentAngle.add(getAbsoluteAngle(spot));
    }

    // The absolute angle of a spot relative to spot 0
    public Angle getAbsoluteAngle(int spot) {
        assert spot < NUM_SPOTS : "Spot must be less than " + spot;
        return Angle.fromDegrees(360f * spot / NUM_SPOTS);
    }

    private int findNearestSpotIndex(Angle query, BallColor matchColorOrNull) {
        int nearestSpot = 0;
        double smallestGap = 180;
        for (int spot = 0; spot < NUM_SPOTS; spot++) {
            if (matchColorOrNull != null && ballColors[spot] != matchColorOrNull) continue;

            double gap = query.absGap(getRelativeAngle(spot)).toDegrees();
            if (gap < smallestGap) {
                smallestGap = gap;
                nearestSpot = spot;
            }
        }
        return nearestSpot;
    }

    public int getNearestSpotIndex(Angle query) {
        return findNearestSpotIndex(query, null);
    }

    public int getNearestSpotIndex(Angle query, BallColor matchColor) {
        return findNearestSpotIndex(query, matchColor);
    }

    // Sign of power is direction of spin
    public void spin(double power) {
        turner.setRunMode(CRServoEx2.RunMode.RawPower);
        turner.set(power);
    }

    public boolean isAtAngle(Angle angle) {
        return currentAngle.add(angle).abs().toDegrees()
                < finishedThreshold.abs().toDegrees();
    }

    public boolean isAtSpot(int spot) {
        assert spot < NUM_SPOTS : "Spot must be less than " + spot;
        return getRelativeAngle(spot).abs().toDegrees()
                < finishedThreshold.abs().toDegrees();
    }

    public void removeBall(int spot) {
        assert spot < NUM_SPOTS : "Spot must be less than " + spot;
        ballColors[spot] = BallColor.NONE;
    }

    public void goToAngle(Angle angle, CRServoEx2.RunMode runMode) {
        turner.setRunMode(runMode);
        if (runMode == CRServoEx2.RunMode.OptimizedPositionalControl) {
            turner.set(-angle.toDegrees());
        } else {
//            throw new IllegalStateException("" + currentAngle.sub(angle).sign());
            test = currentAngle.add(angle);
            turner.set(-currentAngle.add(angle).sign() * shootRawPower); // Careful signs work out
        }
    }

    public void goToSpot(int spot, CRServoEx2.RunMode runMode) {
        goToAngle(getAbsoluteAngle(spot), runMode);
    }
}
