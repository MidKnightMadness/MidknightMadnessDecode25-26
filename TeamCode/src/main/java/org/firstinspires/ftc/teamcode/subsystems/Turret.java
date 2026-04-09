package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Range;

@Config
@Configurable
public class Turret extends SubsystemBase {
    // 0 degrees is facing intake
    // Clockwise is negative
    public static Range servoRange = new Range(0, 1);
    public static Range angleRange = new Range(Math.toRadians(-180), Math.toRadians(180));
    public static boolean leftInverted = true, rightInverted = true;

    public static Angle finishedThreshold = Angle.fromDegrees(5); // TODO: Change to 15 for auto?
    public static Angle strictFinished = Angle.fromDegrees(2);

    public ServoExGroup servos;
    public IncrementalEncoder encoder;

    //turret assumes the servos are at 0.5 and should be facing opposite direction as intake
    Angle currentAngle;
    double servoPosition;

    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        encoder = new IncrementalEncoder(
                hardwareMap, ConfigNames.turretEncoder, 8192, AngleUnit.RADIANS
        ).setReversed(false);

        servos = new ServoExGroup(
                new ServoEx(hardwareMap, ConfigNames.turretServoLeft).setInverted(leftInverted),
                new ServoEx(hardwareMap, ConfigNames.turretServoRight).setInverted(rightInverted)
        );

        if (resetEncoder) reset();
    }

    public void reset() {
        servos.set(0.5);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        resetEncoderPosition();
    }

    public void resetEncoderPosition(){
        encoder.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.zero();
    }


    @Override
    public void periodic(){
        currentAngle = Angle.fromRadians(encoder.getAngleUnnormalized());
        servoPosition = servos.get();
    }

    public boolean isAtPosition(double position, boolean strict){
        double angle = servoRange.toRange(position, angleRange);
        return isAtAngle(Angle.fromRadians(angle), strict);
    }

    public boolean isAtAngle(Angle angle, boolean strict){
        return strict ?
                currentAngle.atAngle(angle, strictFinished) :
                currentAngle.atAngle(angle, finishedThreshold);
    }

    // Takes in target angle
    private Angle optimizeAngle(Angle targetAngle) {
        Angle bestCandidate = new Angle();
        Angle targetAngleWrapped = targetAngle.wrap();
        Angle minGap = Angle.fromRadians(Double.MAX_VALUE);
        Angle[] candidates = {
                targetAngleWrapped,
                targetAngleWrapped.sub(Angle.fromRadians(2 * Math.PI)),
                targetAngleWrapped.add(Angle.fromRadians(2 * Math.PI))
        };

        for (Angle candidate : candidates) {
            if (!angleRange.contains(candidate.toRadians())) continue;
            Angle gap = currentAngle.sub(candidate).abs();
            if (gap.less(minGap)) {
                minGap = gap;
                bestCandidate = candidate;
            }
        }

        return bestCandidate;
    }

    public void setPosition(double position) {
        servos.set(position);
    }

    public void setAngle(Angle angle){
        setPosition(angleRange.toRange(angle.toRadians(), servoRange));
    }

    public void setAngleOptimized(Angle target){
        setAngle(optimizeAngle(target));
    }

    public double getTargetPosition() {
        return servoPosition;
    }

    public Angle getTargetAngle() {
        return Angle.fromRadians(servoRange.toRange(getTargetPosition(), angleRange));
    }

    public double getPosition() {
        return servoRange.toRange(getAngle().toRadians(), angleRange);
    }

    public Angle getAngle() {
        return currentAngle;
    }

    public double getTotalRangeDegrees(){
        return angleRange.range();
    }
}