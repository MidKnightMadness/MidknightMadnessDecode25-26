package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.old.opModes.BackSixBallAutoFSM;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.BallColor;

@Configurable
public class Spindexer extends SubsystemBase {
    static class BallSensor {
        BallDetector sensor;
        Angle angle;

        BallSensor(BallDetector sensor, Angle angle) {
            this.sensor = sensor;
            this.angle = angle;
        }
    }

    // 0 is defined as the position of the shooter
    public static Angle detectRange = Angle.fromDegrees(40); // How far off from the center of the spot that you detect. You don't want to trust measurements that are too off from the center
    public static Angle shooterAngle = Angle.fromDegrees(0);
    public static Angle inColorSensorAngle = Angle.fromDegrees(180);
    public static Angle outColorSensorAngle = Angle.fromDegrees(0);
    public static Angle spotZeroReading = Angle.fromDegrees(0); // Raw encoder reading when spot 0 is aligned with shooter
    public static Angle finishedThreshold = Angle.fromDegrees(20); // Threshold at which it's finished turning to a spot

    CRServoEx turner;
    BallSensor[] ballSensors;
    Angle currentAngle;
    BallColor[] ballColors;

    private static final int NUM_SPOTS = 3;

    public Spindexer(HardwareMap hardwareMap) {
        AbsoluteAnalogEncoder turnerEncoder = new AbsoluteAnalogEncoder(
                hardwareMap,
                "turnerEncoder",
                10, // TODO: I don't know what ts does so fix it later
                AngleUnit.DEGREES
        );
        turner = new CRServoEx(
                hardwareMap, "turner",
                turnerEncoder, CRServoEx.RunMode.RawPower
        );
        ballSensors = new BallSensor[] {
                new BallSensor(
                        new BallDetector(hardwareMap, "inColorSensor"),
                        inColorSensorAngle
                ),
                new BallSensor(
                        new BallDetector(hardwareMap, "outColorSensor"),
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
            Angle sensorAngle = ballSensor.angle;
            int spot = getNearestSpotIndex(ballSensor.angle);
            double gap = getSpotAngle(spot).absGap(sensorAngle).toDegrees();
            if (gap < detectRange.toDegrees()) {
                ballColors[spot] = ballSensor.sensor.readColor();
            }
        }
    }

    public int[] getMotifSequence(MotifEnums.Motif motif) {
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
                momentum = getSpotAngle(spot).sign();
            }
        } else {
            int momentum = 0; // 1 or -1 for CW or CCW respectively
            for (int i = 0; i < greenCount + purpleCount; i++) {
                int spot;
                if (i == 0) {
                    spot = getNearestSpotIndex(Angle.fromDegrees(0));
                } else {
                    spot = (sequence[i - 1] + momentum) % NUM_SPOTS;
                    if (ballColors[spot] != BallColor.PURPLE) spot++; // You only need to check 2 spots total
                }

                sequence[i] = spot;
                momentum = getSpotAngle(spot).sign();
            }
        }
        return sequence;
    }

    // The current angle of a spot relative to the outtake
    public Angle getSpotAngle(int spot) {
        assert spot < NUM_SPOTS : "Spot must be less than " + spot;
        return currentAngle.add(Angle.fromDegrees(360f * spot / NUM_SPOTS));
    }

    // Get index of nearest spot
    public int getNearestSpotIndex(Angle query, boolean ignoreNone) {
        int nearestSpot = 0;
        double smallestGap = 180;
        for (int spot = 0; spot < NUM_SPOTS; spot++) {
            if (ignoreNone && ballColors[spot] == BallColor.NONE) break;
            double gap = query.absGap(getSpotAngle(spot)).toDegrees();
            if (gap < smallestGap) {
                smallestGap = gap;
                nearestSpot = spot;
            }
        }
        return nearestSpot;
    }

    // Get index of nearest spot
    public int getNearestSpotIndex(Angle query) {
        return getNearestSpotIndex(query, false);
    }

    // Sign of power is direction of spin
    public void spin(double power) {
        turner.setRunMode(CRServoEx.RunMode.RawPower);
        turner.set(power);
    }

    public boolean isAtSpot(int spot) {
        assert spot < NUM_SPOTS : "Spot must be less than " + spot;
        return currentAngle.absGap(getSpotAngle(spot).neg()).toDegrees()
                < finishedThreshold.abs().toDegrees();
    }

    public void goToAngle(Angle angle) {
        turner.setRunMode(CRServoEx.RunMode.OptimizedPositionalControl);
        turner.set(-angle.toDegrees()); // Rotate in opposite direction to get to angle
    }

    public void goToSpot(int spot) {
        goToAngle(getSpotAngle(spot).neg());
    }
}
