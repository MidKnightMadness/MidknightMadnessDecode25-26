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
@Disabled
@TeleOp(name = "Servo Setting", group = "General")
public class ServoSetting extends OpMode {
    Servo pushUpServo;
    double setPos = 0.5;
    double increment = 0.001;
    ServoImplEx spindexerServo;
    Servo currentServo = pushUpServo;
    boolean useSpindexer = true;

    @Override
    public void init() {
        pushUpServo = hardwareMap.get(Servo.class, ConfigNames.pushUpServo);
        spindexerServo = hardwareMap.get(ServoImplEx.class, ConfigNames.turner);
        spindexerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        currentServo = pushUpServo;
    }

    @Override
    public void loop() {


        if(useSpindexer) {
            spindexerServo.setPosition(setPos);
        } else{
            pushUpServo.setPosition(setPos);
        }

        if(gamepad1.dpad_up){
            setPos+= increment;
        }
        if(gamepad1.dpad_down){
            setPos -= increment;
        }

        if(gamepad1.dpadLeftWasPressed()){
            useSpindexer = !useSpindexer;
        }


        telemetry.addData("Current Pos", setPos);
//        telemetry.addData("Current Servo", currentServo.toString());1
        telemetry.update();
    }
}
