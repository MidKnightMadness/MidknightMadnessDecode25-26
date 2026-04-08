package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Configurable
@TeleOp(name = "Servo Setting", group = "General")
public class ServoSetting extends OpMode {
    Servo pushUpServo;
    Servo turret1Servo;
    Servo turret2Servo;
    double setPos = 0.5;
    double increment = 0.001;
    ServoImplEx spindexerServo;
    Servo currentServo = pushUpServo;
    Servo stopItServo;
    boolean useSpindexer = false;

    @Override
    public void init() {
        pushUpServo = hardwareMap.get(Servo.class, ConfigNames.pushUpServo);
        spindexerServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turner);
        spindexerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        stopItServo = hardwareMap.get(Servo.class, ConfigNames.stopItServo);

        currentServo = stopItServo;
//        turret1Servo = hardwareMap.get(Servo.class, ConfigNames.turretServoLeft);
//        turret2Servo = hardwareMap.get(Servo.class, ConfigNames.turretServoRight);
    }

    @Override
    public void loop() {

        stopItServo.setPosition(setPos);

        if(gamepad1.dpad_up){
            setPos+= increment;
        }
        if(gamepad1.dpad_down){
            setPos -= increment;
        }

        if(gamepad1.dpadLeftWasPressed()){
            useSpindexer = !useSpindexer;
        }

        if(gamepad1.leftBumperWasPressed()){
            spindexerServo.setPosition(0);
        }
        if(gamepad1.rightBumperWasPressed()){
            spindexerServo.setPosition(1);
        }

        telemetry.addData("Current Pos", setPos);
//        telemetry.addData("Current Servo", currentServo.toString());1
        telemetry.update();
    }
}
