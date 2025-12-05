package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localization.kalmanFilter.KalmanPinpointAprilConstants;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Configurable
public class ConstantsBot {
    public static boolean motifIsBusy = true;
    public static GoBildaPinpointDriver.DeviceStatus deviceStatus = GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-57)//TUNED
            .lateralZeroPowerAcceleration(-85)//TUNED
            // Translational PIDF
            .translationalPIDFSwitch(4)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.04, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.04, 0.03))
            // Heading PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.02, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.01, 0.02))
            // Drive PIDF
            .drivePIDFSwitch(15)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.003, 0.6, 0.02))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.005, 0.6, 0.03))
            .centripetalScaling(0.0005)
            .mass(15.5);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.5, 1);

    //competition
    public static PinpointConstants pinpointLocalizerConstants = new PinpointConstants()
            .forwardPodY(-115/25.4)
            .strafePodX(173.7/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(ConfigNames.pinpoint)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PinpointConstants customPinpointLocalizerConstants = new PinpointConstants()
            .forwardPodY(-115/25.4)
            .strafePodX(173.7/25.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(ConfigNames.pinpoint)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static KalmanPinpointAprilConstants mergedLocalizerConstants = new KalmanPinpointAprilConstants()
            .setIMUName(ConfigNames.imu)
            .setLimelightName(ConfigNames.limelight)
            .setLeftPipelineNum(0)
            .setRightPipelineNum(2)
            .setStartPipeline(2)
            .setPinpointHardwareConfig(ConfigNames.pinpoint)
            .setQ(0.01)
            .setR(2)
            .setMotifTrue(true)
            .setXOffset(138.874)
            .setYOffset(33)
            .setDistUnit(DistanceUnit.MM)
            .setStartPipeline(1)
            .setEncoderXDir(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setEncoderYDir(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(68.2)
            .yVelocity(50.0)
            .rightFrontMotorName(ConfigNames.FR)
            .rightRearMotorName(ConfigNames.BR)
            .leftRearMotorName(ConfigNames.BL)
            .leftFrontMotorName(ConfigNames.FL)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createNoLocalizerFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
    public static Follower createPinpointFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointLocalizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
    public static Follower createPinpointFollowerCustom(HardwareMap hardwareMap, Pose startPose) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizerCustom(customPinpointLocalizerConstants, startPose)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }


    public static Follower createKalmanPinpointAprilFollower(HardwareMap hardwareMap, Pose startPose, Telemetry telemetry){//global startPose
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mergedKalmanLocalizer(mergedLocalizerConstants, startPose, telemetry)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();


    }


}

