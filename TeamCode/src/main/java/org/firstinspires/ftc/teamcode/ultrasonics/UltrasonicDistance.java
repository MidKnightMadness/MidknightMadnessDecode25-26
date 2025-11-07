package org.firstinspires.ftc.teamcode.ultrasonics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.locks.Lock;

public class UltrasonicDistance {

    double leftDist;
    double rightDist;
    public static double distBetween  = 5;//INCH
    // Initialize IMU
    // imu = hardwareMap.get(IMU.class, "imu");

    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private I2cDevice sensorLeft;
    private I2cDevice sensorRight;

    // I²C addresses for each sensor (change right one if needed)
    private static final I2cAddr SENSOR_ADDRESS_LEFT  = I2cAddr.create7bit(0x70);
    private static final I2cAddr SENSOR_ADDRESS_RIGHT = I2cAddr.create7bit(0x71);

    // Start register for distance data
    private static final int DISTANCE_REG = 0x02;

    private byte[] readCacheLeft, readCacheRight;
    private Lock readCacheLockLeft, readCacheLockRight;

    // IMU (built into Rev Hub)
    private IMU imu;
    public UltrasonicDistance(HardwareMap hardwareMap){
        sensorLeft = hardwareMap.i2cDevice.get("ultrasonicLeft");
        sensorRight = hardwareMap.i2cDevice.get("ultrasonicRight");

        // Left sensor setup
        readCacheLeft = sensorLeft.getI2cReadCache();
        readCacheLockLeft = sensorLeft.getI2cReadCacheLock();
        sensorLeft.enableI2cReadMode(SENSOR_ADDRESS_LEFT, DISTANCE_REG, 2);
        sensorLeft.setI2cPortActionFlag();
        sensorLeft.writeI2cCacheToController();

        // Right sensor setup
        readCacheRight = sensorRight.getI2cReadCache();
        readCacheLockRight = sensorRight.getI2cReadCacheLock();
        sensorRight.enableI2cReadMode(SENSOR_ADDRESS_RIGHT, DISTANCE_REG, 2);
        sensorRight.setI2cPortActionFlag();
        sensorRight.writeI2cCacheToController();

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoFacingDirection, usbFacingDirection)));
        imu.resetYaw();

    }


    double imuRobotHeading;
    double ultrasonicAngle;
    double predictedFieldAngle;

    public void update(){
        sensorLeft.readI2cCacheFromController();
        sensorRight.readI2cCacheFromController();


        // Left Sensor
        readCacheLockLeft.lock();
        try {
            int high = readCacheLeft[4] & 0xFF;
            int low = readCacheLeft[5] & 0xFF;
            leftDist = (high << 8) | low;
        } finally {
            readCacheLockLeft.unlock();
        }

        // Right Sensor
        readCacheLockRight.lock();
        try {
            int high = readCacheRight[4] & 0xFF;
            int low = readCacheRight[5] & 0xFF;
            rightDist = (high << 8) | low;
        } finally {
            readCacheLockRight.unlock();
        }


        imuRobotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        ultrasonicAngle = Math.atan2(rightDist - leftDist, distBetween);//inches
        predictedFieldAngle = getNormalizedAngle(Math.toRadians(270) - ultrasonicAngle);

    }

    public double getNormalizedAngle(double val){//normalize to 0 -> 2PI
        double remainder = val % (2 * Math.PI);
        if(remainder < 0){
            remainder += 2 * Math.PI;
        }
        return remainder;
    }
    public double getDist1(){
        return leftDist;
    }

    public double getDist2(){
        return rightDist;
    }


    public double getRobotHeadingRad(){
        return predictedFieldAngle;
    }

    //
    public double getRobotHeadingDeg(){
        return predictedFieldAngle * 180 / Math.PI;
    }
    public double getIMUHeadingDeg(){
        return imuRobotHeading;
    }
}
