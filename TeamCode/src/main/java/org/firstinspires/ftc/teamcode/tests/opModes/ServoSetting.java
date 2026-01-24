package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "Servo Setting", group = "General")
public class ServoSetting extends OpMode {
    Servo pushUpServo;
    double setPos = 0.5;
    double increment = 0.001;

    Servo currentServo = pushUpServo;
    @Override
    public void init() {
        pushUpServo = hardwareMap.get(Servo.class, ConfigNames.pushUpServo);
        currentServo = pushUpServo;
    }

    @Override
    public void loop() {
        pushUpServo.setPosition(setPos);

        if(gamepad1.dpad_up){
            setPos+= increment;
        }
        if(gamepad1.dpad_down){
            setPos -= increment;
        }


        telemetry.addData("Current Pos", setPos);
//        telemetry.addData("Current Servo", currentServo.toString());1
        telemetry.update();
    }
}
