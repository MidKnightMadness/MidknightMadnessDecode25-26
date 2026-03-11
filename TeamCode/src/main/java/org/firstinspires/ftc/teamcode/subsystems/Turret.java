package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.IncrementalEncoderNonCR;
import org.firstinspires.ftc.teamcode.util.AngleNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
public class Turret extends SubsystemBase {

    private ServoImplEx leftServo;
    private ServoImplEx rightServo;
    private IncrementalEncoderNonCR encoder;

    // Encoder config
    public static double degreesPerRevolution = 300;

    // Servo limits (0.0 to 1.0 maps to 0-300°)
    public static double minPos = 0.0;
    public static double maxPos = 1.0;
    Servo.Direction directionLeft = Servo.Direction.FORWARD;
    Servo.Direction directionRight = Servo.Direction.FORWARD;

    public static double startPositionLeft = 0.5;
    public static double startPositionRight = 0.5;
    public static AngleNonCR finishedThreshold = AngleNonCR.fromDegrees(5);//TODO: Change to 15 for auto?
    public static AngleNonCR strictFinished = AngleNonCR.fromDegrees(2);
    //turret assumes the servos are at 0.5 and should be facing opposite direction as intake
    AngleNonCR currentAngle;
    double currLeftPosition;
    double currRightPosition;
    double startHeadingTurret = 90;
    public Turret(HardwareMap hardwareMap, boolean setStart) {

        leftServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoLeft);
        rightServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoRight);
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftServo.setDirection(directionLeft);
        rightServo.setDirection(directionRight);

        if(setStart) {
            leftServo.setPosition(startPositionLeft);
            rightServo.setPosition(startPositionRight);
        }

        encoder = new IncrementalEncoderNonCR(
                hardwareMap, ConfigNames.turretEncoder, 8192, AngleUnit.DEGREES
        ).setReversed(false);

//        lastTime = System.nanoTime();
    }


    @Override
    public void periodic(){
        currentAngle = AngleNonCR.fromDegrees(encoder.getAngle());
        currLeftPosition = leftServo.getPosition();
        currRightPosition = rightServo.getPosition();
    }

    public void setStartHeadingTurret(double startHeadingTurret){
        this.startHeadingTurret = startHeadingTurret;
    }

    public double getLastSetTurretAngle() {
        return currentAngle.getValue();
    }

    public double getLastSetTurretPosition(){
        return currentAngle.getValue() / degreesPerRevolution;
    }

    public boolean isAtPosition(double position){
        double angle = position * degreesPerRevolution;
        return isAtAngle(AngleNonCR.fromDegrees(angle), finishedThreshold);
    }
    public boolean isAtPositionStrict(double position){
        double angle = position * degreesPerRevolution;
        return isAtAngleStrict(AngleNonCR.fromDegrees(angle));
    }
    public boolean isAtAngle(AngleNonCR angle, AngleNonCR finishedThreshold) {
        return angle.diff(currentAngle).toDegrees() < finishedThreshold.abs().toDegrees();
    }

    public boolean isAtAngleStrict(AngleNonCR angle){
        return angle.diff(currentAngle).toDegrees() < strictFinished.abs().toDegrees();
    }



    public void update(double targetHeading) {
        setServos(targetHeading / degreesPerRevolution);
    }

    public void setServos(double servoPosition){

        leftServo.setPosition(servoPosition);
        rightServo.setPosition(servoPosition);
    }

    // Map turret angle to servo positions (0-300° -> 0.0-1.0)
    private void moveServos(double turretAngle) {
        double servoPos = Range.clip(turretAngle / degreesPerRevolution, minPos, maxPos);

        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }
    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}