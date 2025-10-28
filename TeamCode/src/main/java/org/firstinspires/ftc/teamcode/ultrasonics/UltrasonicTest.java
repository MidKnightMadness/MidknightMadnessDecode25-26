package org.firstinspires.ftc.teamcode.ultrasonics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.locks.Lock;


@Config
@TeleOp(name = "Dual Maxbotix I2C + IMU", group = "Sensor")
public class UltrasonicTest extends OpMode {

    double distanceLeft;
    double distanceRight;
    double imuHeading;
    double ultrasonicHeading;
    UltrasonicDistance ultrasonicDistance;
    @Override
    public void init() {
        ultrasonicDistance = new UltrasonicDistance(hardwareMap);
    }


    @Override
    public void loop() {
        ultrasonicDistance.update();
        imuHeading = ultrasonicDistance.getIMUHeadingDeg();
        ultrasonicHeading = ultrasonicDistance.getRobotHeadingDeg();

        distanceLeft = ultrasonicDistance.getDist1();
        distanceRight = ultrasonicDistance.getDist2();

        telemetry.addData("Left Distance (mm)", distanceLeft);
        telemetry.addData("Right Distance (mm)", distanceRight);
        telemetry.addData("Heading (Deg) IMU", imuHeading);
        telemetry.addData("Heading (Deg) Ultrasonic", ultrasonicHeading);
        telemetry.update();

    }
}
