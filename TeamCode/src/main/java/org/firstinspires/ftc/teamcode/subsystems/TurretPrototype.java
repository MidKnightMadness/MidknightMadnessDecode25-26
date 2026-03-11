package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class TurretPrototype {

    private Servo leftServo;
    private Servo rightServo;
    private DcMotorEx encoder;

    // PID constants (tune these)
    public static double kP = 0.01;
    public static double kD = 0.0005;

    // Encoder config
    public static double ticksPerDegree = 1.493; // example: 537.6 ticks per rev / 360 deg

    // Servo limits (0.0 to 1.0 maps to 0-300°)
    public static double minPos = 0.0;
    public static double maxPos = 1.0;

    // Deadband to prevent jitter
    public static double deadband = 0.5; // degrees

    private double lastError = 0;
    private long lastTime;

    private double targetHeading = 0;

    public TurretPrototype(HardwareMap hardwareMap) {

        leftServo = hardwareMap.get(Servo.class, "turretServoLeft");
        rightServo = hardwareMap.get(Servo.class, "turretServoRight");

        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // initialize servos to center (150 degrees)
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        lastTime = System.nanoTime();
    }

    // get the current turret angle from encoder (in degrees)
    public double getTurretAngle() {
        return encoder.getCurrentPosition() / ticksPerDegree;
    }

    public void setTargetHeading(double heading) {
        targetHeading = heading;
    }

    public void update(double robotHeading) {

        double turretAngle = getTurretAngle();

        // Target turret angle relative to robot
        double targetTurretAngle = normalize(targetHeading - robotHeading);

        double error = targetTurretAngle - turretAngle;

        // Deadband
        if (Math.abs(error) < deadband) {
            moveServos(turretAngle); // hold current position
            return;
        }

        // Calculate loop time
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double derivative = (error - lastError) / dt;
        lastError = error;

        // PD output (in degrees)
        double outputAngle = turretAngle + (kP * error + kD * derivative);

        moveServos(outputAngle);
    }

    // Map turret angle to servo positions (0-300° -> 0.0-1.0)
    private void moveServos(double turretAngle) {
        double servoPos = Range.clip(turretAngle / 300.0, minPos, maxPos);

        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }
    private double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}