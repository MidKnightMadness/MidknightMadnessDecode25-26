package org.firstinspires.ftc.teamcode.Localization.KalmanFilter;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Dictionary;

@Config
@Configurable
public class KalmanPinpointAprilConstants {
    public double Q = 0.01;
    public double R = 2;
    public int leftPipelineNum = 0;
    public int rightPipelineNum = 2;
    public int startPipeline = 2;
    public String LIMELIGHT_NAME = "limelight";
    public boolean MOTIF_DETECTION = true;

    //------------------------------------------------------------

    public String PINPOINT_NAME = "pinpoint";
    public double xOffset = 138.874;
    public double yOffset = 33;
    public GoBildaPinpointDriver.EncoderDirection xDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public DistanceUnit distUnit = DistanceUnit.INCH;
    public String IMU_NAME = "imu";
    RevHubOrientationOnRobot imuOrientation;
    public KalmanPinpointAprilConstants setQ(double Q){
        this.Q = Q;
        return this;
    }
    public KalmanPinpointAprilConstants setR(double R){
        this.R = R;
        return this;
    }

    public KalmanPinpointAprilConstants setLeftPipelineNum(int leftPipeNum){
        this.leftPipelineNum = leftPipeNum;
        return this;
    }
    public KalmanPinpointAprilConstants setRightPipelineNum(int rightPipeNum){
        this.rightPipelineNum = rightPipeNum;
        return this;
    }
    public KalmanPinpointAprilConstants setXOffset(double xOffset){
        this.xOffset = xOffset;
        return this;
    }

    public KalmanPinpointAprilConstants setYOffset(double yOffset){
        this.yOffset = yOffset;
        return this;
    }

    public KalmanPinpointAprilConstants setPinpointHardwareConfig(String pinpointHardwareName) {
        this.PINPOINT_NAME = pinpointHardwareName;
        return this;
    }
    public KalmanPinpointAprilConstants setIMU(RevHubOrientationOnRobot orientations){
        this.imuOrientation = orientations;
        return this;
    }

    public KalmanPinpointAprilConstants setIMUName(String imuHardwareName){
        this.IMU_NAME = imuHardwareName;
        return this;
    }

    public KalmanPinpointAprilConstants setDistUnit(DistanceUnit d){
        this.distUnit = d;
        return this;
    }

    public KalmanPinpointAprilConstants setEncoderXDir(GoBildaPinpointDriver.EncoderDirection dir){
        this.xDir = dir;
        return this;
    }
    public KalmanPinpointAprilConstants setEncoderYDir(GoBildaPinpointDriver.EncoderDirection dir){
        this.yDir = dir;
        return this;
    }

    public KalmanPinpointAprilConstants setLimelightName(String limelightName){
        this.LIMELIGHT_NAME = limelightName;
        return this;
    }

    public KalmanPinpointAprilConstants setStartPipeline(int num){
        this.startPipeline = num;
        return this;
    }

    public KalmanPinpointAprilConstants setMotifTrue(boolean val){
        this.MOTIF_DETECTION = true;
        return this;
    }

}
