package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoderNonCR;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
//Turret Range [-halfRange, halfRange] -> [0, 1], middle pt of 0.5
public class Turret extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    private IncrementalEncoderNonCR encoder;
    public static double totalRangeRadians = 400;
    public static double halfRange = totalRangeRadians / 2.0;
    //if degrees per revolution is 420, 210 degrees from turret(180 degrees position), -210 from turret
    Servo.Direction directionLeft = Servo.Direction.FORWARD;
    Servo.Direction directionRight = Servo.Direction.FORWARD;
    public static AngleNonCR finishedThreshold = AngleNonCR.fromDegrees(5);//TODO: Change to 15 for auto?
    public static AngleNonCR strictFinished = AngleNonCR.fromDegrees(2);
    public static Angle offsetAngle = Angle.fromDegrees(90);
    //turret assumes the servos are at 0.5 and should be facing opposite direction as intake
    AngleNonCR currentAngle;
    double currLeftPosition;
    double currRightPosition;
    double startHeadingTurret = 90;
    public static double cachingTolerance = 0.001;
    double lastPosLeft = -1;
    double lastPosRight = -1;
    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        encoder = new IncrementalEncoderNonCR(
                hardwareMap, ConfigNames.turretEncoder, 8192, AngleUnit.RADIANS, resetEncoder
        ).setReversed(false);

        leftServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoLeft);
        rightServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoRight);
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftServo.setDirection(directionLeft);
        rightServo.setDirection(directionRight);

        if(resetEncoder){
            //set to 0.5 to make sure both sides have same # of degrees
            leftServo.setPosition(servoCenter);
            rightServo.setPosition(servoCenter);
            try{
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            resetEncoderPosition();
        }
    }



    //reset encoder position to 0(pedro field zero)
    public void resetEncoderPosition(){
        encoder.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void periodic(){
        //current angle returns from -210 to 210
        currentAngle = AngleNonCR.fromRadians(encoder.getAngleUnormalizedEncoder());
        currLeftPosition = leftServo.getPosition();
        currRightPosition = rightServo.getPosition();
    }



    public IncrementalEncoderNonCR getEncoder(){
        return encoder;
    }
    public boolean isAtPosition(double position, boolean strict){
        double angle = position * totalRangeRadians;
        return isAtAngle(AngleNonCR.fromRadians(angle), strict);
    }

    public boolean isAtAngle(AngleNonCR angle, boolean strict){
        return Math.abs(angle.diff(currentAngle).toRadians()) <
                (strict ? strictFinished.toRadians() : finishedThreshold.toRadians());
    }



    public double getServoLeftPosition(){
        return leftServo.getPosition();
    }
    public double getServoRightPosition(){
        return rightServo.getPosition();
    }

    /**
    *@params: angleNormalized 0–360 degrees
    */
    public boolean isAtAngle(Angle angleNormalized){
        double target = angleNormalized.toRadians(); // 0–2PI
        double current = currentAngle.getValue();    // -halfRange -> +halfRange

        double error = shortestAngleDifference(target, current);

        return Math.abs(error) < finishedThreshold.toRadians();
    }

    private double shortestAngleDifference(double target, double current){
        // bring current into 0–360 for computation
        double current360 = (current + 2 * Math.PI) % 2 * Math.PI;

        // compute difference
        double diff = target - current360;
        if (diff > Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }



    public static double centerAngle = 0;
    public static double servoCenter = 0.5;
    public double angleToServo(AngleNonCR angle){
        return servoCenter + angle.add(offsetAngle).toRadians() / totalRangeRadians;
    }

    public double servoToAngle(double servoPosition){
        return Angle.fromRadians((servoPosition - servoCenter) * totalRangeRadians).sub(offsetAngle).toRadians();
    }

    //takes in an angle from 0 - 2PI(target angle) and current angle(-half, + half) and finds optimal angle to set servo to
    private double angleToServoOptimized(Angle targetAngle, AngleNonCR currentAngle) {
        double current = currentAngle.getValue();
        targetAngle.add(offsetAngle);

        double target = ((targetAngle.getValue() + Math.PI) % (2 * Math.PI)) - Math.PI;

        double minAngle = -halfRange;
        double maxAngle = +halfRange;

        //candidates +-2PI
        double[] candidates = {target, target + 2 * Math.PI, target - 2 * Math.PI};
        double bestCandidate = current; //default
        double minDistance = Double.MAX_VALUE;

        for (double c : candidates) {
            if (c >= minAngle && c <= maxAngle) {
                double distance = Math.abs(c - current);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestCandidate = c;
                }
            }
        }
        return angleToServo(AngleNonCR.fromRadians(bestCandidate));
    }
    private void setServos(AngleNonCR targetTurretAngle){
        setServos(angleToServo(targetTurretAngle));
    }

    public void setServos(double servoPosition){
        if(Math.abs(servoPosition - lastPosLeft) > cachingTolerance || lastPosLeft == -1){
            leftServo.setPosition(servoPosition);
            lastPosLeft = servoPosition;
        }
        if(Math.abs(servoPosition - lastPosRight) > cachingTolerance || lastPosRight == -1){
            rightServo.setPosition(servoPosition);
            lastPosRight = servoPosition;
        }
    }

    public void setAngle(AngleNonCR angle){
        setServos(angleToServo(angle));
    }

    public void setAngleOptimized(Angle target){
        setServos(angleToServoOptimized(target, currentAngle));
    }




}