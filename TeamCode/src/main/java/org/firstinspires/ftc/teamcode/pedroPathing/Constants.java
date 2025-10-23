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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Localization.KalmanFilter.KalmanPinpointAprilConstants;
import org.firstinspires.ftc.teamcode.Localization.FollowerBuilder;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-52)
            .lateralZeroPowerAcceleration(-87.8)
            // Translational PIDF
            .translationalPIDFSwitch(4)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.02, 0.03))
            // Heading PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.02))
            // Drive PIDF
            .drivePIDFSwitch(15)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.003, 0.6, 0.03))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.003, 0.6, 0.03))
            .centripetalScaling(0.0005)
            .mass(11.80);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.1, 1);

    public static PinpointConstants pinpointLocalizerConstants = new PinpointConstants()
            .forwardPodY(-5.875)
            .strafePodX(1.50)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static KalmanPinpointAprilConstants mergedLocalizerConstants = new KalmanPinpointAprilConstants()
            .setIMUName("imu")
            .setLimelightName("limelight")
            .setLeftPipelineNum(0)
            .setRightPipelineNum(2)
            .setPinpointHardwareConfig("pinpoint")
            .setQ(0.01)
            .setR(2)
            .setMotifTrue(true)
            .setXOffset(138.874)
            .setYOffset(33)
            .setDistUnit(DistanceUnit.MM)
            .setStartPipeline(2)
            .setEncoderXDir(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .setEncoderYDir(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(78)
            .yVelocity(54.5)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createPinpointFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointLocalizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }

    public static Follower createKalmanPinpointAprilFollower(HardwareMap hardwareMap, Pose startPose){//global startPose
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mergedKalmanLocalizer(mergedLocalizerConstants, startPose)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();


    }
}