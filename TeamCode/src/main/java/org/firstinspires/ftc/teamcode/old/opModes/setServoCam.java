package org.firstinspires.ftc.teamcode.old.opModes;

import com.qualcomm.robotcore.hardware.Servo;

public class setServoCam {
    public static void setCam(Servo servo, double angle){
        servo.setPosition(angle);//needs testing too see what angle works
    }
}
