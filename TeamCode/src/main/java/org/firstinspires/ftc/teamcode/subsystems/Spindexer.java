package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.sensors.BallDetector;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ExtraFns;

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
            votes = Map.of(
                    BallColor.GREEN, 0.,
                    BallColor.PURPLE, 0.,
                    BallColor.NONE, 0.
            );
        }
    }

    public static double intakeSpinPower = 0.3;
    public static double shootRawPower = 1;

    // 0 is defined as the position of the shooter
    public Angle spotZeroReading = Angle.fromDegrees(0); // Raw encoder reading when spot 0 is aligned with shooter
    public static Angle detectRange = Angle.fromDegrees(40); // How far off from the center of the spot that you detect. You don't want to trust measurements that are too off from the center
    public static Angle inColorSensorAngle = Angle.fromDegrees(180);
    public static Angle outColorSensorAngle = Angle.fromDegrees(0);
    public static Angle finishedThreshold = Angle.fromDegrees(20); // Threshold at which it's finished turning to a spot

    CRServoEx turner;
    BallSensor[] ballSensors;
    Angle currentAngle;
    BallColor[] ballColors;

    private static final int NUM_SPOTS = 3;

    public Spindexer(HardwareMap hardwareMap) {
        AbsoluteAnalogEncoder turnerEncoder = new AbsoluteAnalogEncoder(
                hardwareMap,
                ConfigNames.turnerEncoder,
                10, // TODO: I don't know what ts does so fix it later
                AngleUnit.DEGREES
        );
        turner = new CRServoEx(
                hardwareMap, ConfigNames.turner,
                turnerEncoder, CRServoEx.RunMode.RawPower
        );
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
        ballColors = new BallColor[] { BallColor.NONE, BallColor.NONE, BallColor.NONE };
    }

    @Override
    public void periodic() {
        currentAngle = Angle.fromDegrees(
                turner.getAbsoluteEncoder().getCurrentPosition() + spotZeroReading.toDegrees()
        );
        updateBallColors();
    }

    public void init() {
        spotZeroReading = Angle.fromDegrees(
                turner.getAbsoluteEncoder().getCurrentPosition()
        );
    }

    public CRServoEx getTurner() {
        return turner;
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

    public int[] getOptimalSequence(MotifEnums.Motif motif) {
        int[] sequence = new int[3];
        int greenSpot = -1;
        double greenCount = 0;
        double purpleCount = 0;
        for (int i = 0; i < ballColors.length; i++) {
            if (ballColors[i] == BallColor.GREEN) {
                greenCount++;
                greenSpot = i;
            }
            else if (ballColors[i] == BallColor.PURPLE) purpleCount++;
        }

        if (!motif.equals(MotifEnums.Motif.NONE) && greenCount == 1 && purpleCount == 2) {
            double greenOrder;
            if (motif.equals(MotifEnums.Motif.GPP)) greenOrder = 0;
            else if (motif.equals(MotifEnums.Motif.PGP)) greenOrder = 1;
            else greenOrder = 2;

            int momentum = 0; // 1 or -1 for CW or CCW respectively
            for (int i = 0; i < 3; i++) {
                int spot;
                if (i == greenOrder) spot = greenSpot;
                else {
                    if (i == 0) {
                        spot = getNearestSpotIndex(Angle.fromDegrees(0));
                    } else {
                        spot = (sequence[i - 1] + momentum) % NUM_SPOTS;
                        if (ballColors[spot] != BallColor.PURPLE) spot++; // You only need to check 2 spots total
                    }
                }

                sequence[i] = spot;
                momentum = getRelativeAngle(spot).sign();
            }
        } else {
            int momentum = 0; // 1 or -1 for CW or CCW respectively
            for (int i = 0; i < greenCount + purpleCount; i++) {
                int spot;
                if (i == 0) {
                    spot = getNearestSpotIndex(Angle.fromDegrees(0));
                } else {
                    spot = (sequence[i - 1] + momentum) % NUM_SPOTS;
                    if (ballColors[spot] == BallColor.NONE) spot++; // You only need to check 2 spots total
                }

                sequence[i] = spot;
                momentum = getRelativeAngle(spot).sign();
            }
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

    // Get index of nearest spot
    public int getNearestSpotIndex(Angle query, BallColor matchColor) {
        int nearestSpot = 0;
        double smallestGap = 180;
        for (int spot = 0; spot < NUM_SPOTS; spot++) {
            if (ballColors[spot] == matchColor) break;
            double gap = query.absGap(getRelativeAngle(spot)).toDegrees();
            if (gap < smallestGap) {
                smallestGap = gap;
                nearestSpot = spot;
            }
        }
        return nearestSpot;
    }

    // TODO: remove duplicate function but it's annoying
    // Get index of nearest spot
    public int getNearestSpotIndex(Angle query) {
        int nearestSpot = 0;
        double smallestGap = 180;
        for (int spot = 0; spot < NUM_SPOTS; spot++) {
            double gap = query.absGap(getRelativeAngle(spot)).toDegrees();
            if (gap < smallestGap) {
                smallestGap = gap;
                nearestSpot = spot;
            }
        }
        return nearestSpot;
    }

    // Sign of power is direction of spin
    public void spin(double power) {
        turner.setRunMode(CRServoEx.RunMode.RawPower);
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

    public void goToAngle(Angle angle, CRServoEx.RunMode runMode) {
        turner.setRunMode(runMode);
        if (runMode == CRServoEx.RunMode.OptimizedPositionalControl) {
            turner.set(-angle.toDegrees());
        } else {
            turner.set(currentAngle.sub(angle).sign() * shootRawPower); // Careful signs work out
        }
    }

    public void goToSpot(int spot, CRServoEx.RunMode runMode) {
        goToAngle(getAbsoluteAngle(spot), runMode);
    }
}
