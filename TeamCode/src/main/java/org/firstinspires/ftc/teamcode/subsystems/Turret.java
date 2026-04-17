package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoder;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
//Turret Range [-halfRange, halfRange] -> [0, 1], middle pt of 0.5
public class Turret extends SubsystemBase {
    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    private IncrementalEncoder encoder;
    public static double totalRangeRadians = Math.toRadians(396);
    public static double halfRange = totalRangeRadians / 2.0;
    public static double cappedRangeRadians = Math.toRadians(340);
    //Nihao -MrFiso 4/10/2026 6:12 pm PST
    //if degrees per revolution is 420, 210 degrees from turret, -210 from turret
    Servo.Direction directionLeft = Servo.Direction.REVERSE;
    Servo.Direction directionRight = Servo.Direction.REVERSE;
    public static AngleNonCR finishedThreshold = AngleNonCR.fromDegrees(5);
    public static AngleNonCR strictFinished = AngleNonCR.fromDegrees(2);
    //turret assumes the servos are at 0.5 and robot, turret facing 270 deg global position
    public static double ENCODER_GEAR_RATIO = 24.0 / 60.0;
    AngleNonCR currentAngle;
    double currLeftPosition;
    double currRightPosition;
//    public static double cachingTolerance = 0.001;
    double lastPosLeft = -1;
    double lastPosRight = -1;
    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        encoder = new IncrementalEncoder(
                hardwareMap, ConfigNames.turretEncoder, 8192, AngleUnit.RADIANS
        ).setReversed(true);

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
        currentAngle = AngleNonCR.fromRadians(encoder.getAngleUnnormalized() * ENCODER_GEAR_RATIO);
        currLeftPosition = angleToServo(currentAngle);
        currRightPosition = angleToServo(currentAngle);
    }

    public double getCurrLeftPosition(){
        return currLeftPosition;
    }
    public double getCurrRightPosition(){
        return currRightPosition;
    }
    public AngleNonCR getCurrentAngle(){
        return currentAngle;
    }

    public IncrementalEncoder getEncoder(){
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
        double current360 = (current + 2 * Math.PI) % (2 * Math.PI);

        // compute difference
        double diff = target - current360;
        if (diff > Math.PI) diff -= 2 * Math.PI;
        if (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
    public static double servoCenter = 0.505;
    public double angleToServo(AngleNonCR angle){
        return servoCenter + Math.min(Math.max(-cappedRangeRadians / 2, angle.getValue()), cappedRangeRadians / 2) / totalRangeRadians;
    }
    public double angleToServo(Angle angle){
        return servoCenter + Math.min(Math.max(-cappedRangeRadians / 2, angle.getValue()), cappedRangeRadians / 2) / totalRangeRadians;
    }

    public double servoToAngle(double servoPosition){
        return Angle.fromRadians((servoPosition - servoCenter) * totalRangeRadians).toRadians();
    }

    double minimumWrapChange = Math.toRadians(40);
    //takes in an angle from 0 - 2PI(target angle) and current angle(-half, + half) and finds optimal angle to set servo to
    private double angleToServoOptimized(Angle targetAngle, AngleNonCR currentAngle) {
        double current = currentAngle.getValue();

        double target = targetAngle.getValue() - 3 * Math.PI / 2;
        if(target < - Math.PI) target += 2 * Math.PI;
        //converts 0 - 2PI to center 0 at 3PI/2, + counterclockwise, - clockwise


        double minAngle = -halfRange;
        double maxAngle = +halfRange;

        //candidates +-2PI
        double[] candidates = {target, target + 2 * Math.PI, target - 2 * Math.PI};
        double bestCandidate = current; //default
        double minDistance = Double.MAX_VALUE;

        for (double c : candidates) {
            if (c >= minAngle && c <= maxAngle) {
                double distance = Math.abs(c - current);
                if (distance + minimumWrapChange < minDistance) {
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
//        if(Math.abs(servoPosition - lastPosLeft) > cachingTolerance || lastPosLeft == -1){
            leftServo.setPosition(servoPosition);
//            lastPosLeft = servoPosition;
//        }
//        if(Math.abs(servoPosition - lastPosRight) > cachingTolerance || lastPosRight == -1){
            rightServo.setPosition(servoPosition);
//            lastPosRight = servoPosition;
//        }
    }

    public void setAngle(Angle angle){
        setServos(angleToServo(angle));
    }

    //plug in angle -2PI <-> 2PI angle and restrict to -PI to PI
    public void setFieldAngleToServo(Angle angle){
        double angleValue = angle.getValue();
        if(angleValue < -Math.PI){
            angleValue += 2 * Math.PI;
        }
        if(angleValue > Math.PI){
            angleValue -= 2 * Math.PI;
        }
        setServos(angleToServo(Angle.fromRadians(angleValue)));
    }

    public void setAngleOptimized(Angle target){
        setServos(angleToServoOptimized(target, currentAngle));
    }




}