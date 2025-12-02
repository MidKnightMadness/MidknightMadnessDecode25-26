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
    CRServo servo;

    public static double power = 1;
    @Override
    public void init() {
        servo = new CRServo(hardwareMap, ConfigNames.turner);

    }

    @Override
    public void loop() {
        servo.set(power);
        telemetry.addData("Current Power", servo.get());
    }
}
