package org.firstinspires.ftc.teamcode.tests.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

import org.firstinspires.ftc.teamcode.util.ConfigNames;


@Config
@Configurable
@TeleOp(name = "Continuous Servo Test", group = "Tests")
public class ContinuousServoTest extends OpMode {
    CRServo servoOne;
    CRServo servoTwo;

    public static double firstPower = 1;
    public static double secondPower = 1;
    @Override
    public void init() {
        servoOne = new CRServo(hardwareMap, ConfigNames.turner);
        servoTwo = new CRServo(hardwareMap, ConfigNames.turner2);
    }

    @Override
    public void loop() {
        servoOne.set(firstPower);
        servoTwo.set(secondPower);
        telemetry.addData("First Servo Power", firstPower);
        telemetry.addData("Second Servo Power", secondPower);
    }
}
