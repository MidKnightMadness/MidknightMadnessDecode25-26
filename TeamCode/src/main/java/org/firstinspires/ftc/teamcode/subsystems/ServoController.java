package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoController {

    private final Servo servo;
    private final double minAngle;
    private final double maxAngle;

    public ServoController(Servo servo, double minAngle, double maxAngle) {
        this.servo = servo;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }


    public void setAngle(double angle) {
        if (angle < minAngle) angle = minAngle;
        if (angle > maxAngle) angle = maxAngle;

        double position = (angle - minAngle) / (maxAngle - minAngle);
        servo.setPosition(position);
    }
}
