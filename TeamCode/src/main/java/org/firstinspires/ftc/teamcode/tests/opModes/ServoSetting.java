package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

public class ServoSetting extends OpMode {
    Servo rampServo;
    double setPos = 0.5;
    double increment = 0.05;

    Servo currentServo = rampServo;
    @Override
    public void init() {
        rampServo = hardwareMap.get(Servo.class, ConfigNames.rampServo);
    }

    @Override
    public void loop() {

        currentServo.setPosition(setPos);

        if(gamepad1.dpad_up){
            setPos+= increment;
        }
        if(gamepad1.dpad_down){
            setPos -= increment;
        }


        telemetry.addData("Current Pos", setPos);
        telemetry.addData("Current Servo", currentServo.toString());
    }
}
