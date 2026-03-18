package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "Servo Setting Turret", group = "Turret")
public class ServoSettingTurret extends OpMode {

    ServoImplEx leftServo;
    ServoImplEx rightServo;
    @Override
    public void init() {
        leftServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoLeft);
        rightServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turretServoRight);
        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    boolean first;
    @Override
    public void loop() {
        if(!first) {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
            first = true;
        }
    }
}
