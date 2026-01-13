package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class setServoCam {
    static void setCam(HardwareMap hardwareMap, double angle){
        Servo servo = hardwareMap.get(Servo.class, "limelightServo");
        servo.setPosition(angle);//needs testing too see what angle works
    }
}
