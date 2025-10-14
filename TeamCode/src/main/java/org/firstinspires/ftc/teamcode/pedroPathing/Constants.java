package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-52)
            .lateralZeroPowerAcceleration(-87.8)
            // Translational PIDF
            .translationalPIDFSwitch(4)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.02, 0.01))
            // Heading PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.02))
            // Drive PIDF
            .drivePIDFSwitch(15)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.002, 0.6, 0.03))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.005, 0.6, 0.03))
            .centripetalScaling(0.0005)
            .mass(11.80);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.875)
            .strafePodX(1.50)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.FilteredPIDFCoefficients;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Constants {
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(16.2)
//            .forwardZeroPowerAcceleration(-25.9346931313679598)
//            .lateralZeroPowerAcceleration(-67.342491844080064)
//            .translationalPIDFCoefficients(new PIDFCoefficients(
//                    0.03,
//                    0,
//                    0,
//                    0.015
//            ))
//            .translationalPIDFSwitch(4)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.4,
//                    0,
//                    0.005,
//                    0.0006
//            ))
//            .headingPIDFCoefficients(new PIDFCoefficients(
//                    0.8,
//                    0,
//                    0,
//                    0.01
//            ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
//                    2.5,
//                    0,
//                    0.1,
//                    0.0005
//            ))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.1,
//                    0,
//                    0.00035,
//                    0.6,
//                    0.015
//            ))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.02,
//                    0,
//                    0.000005,
//                    0.6,
//                    0.01
//            ))
//            .drivePIDFSwitch(15)
//            .centripetalScaling(0.0005);
//
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .rightFrontMotorName("FR")
//            .rightRearMotorName("BR")
//            .leftRearMotorName("BL")
//            .leftFrontMotorName("FL")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(78.261926752421046666666666666667)
//            .yVelocity(61.494551922189565);
//
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(0.75)
//            .strafePodX(-6.6)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//    /**
//     These are the PathConstraints in order:
//     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
//     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
//
//     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
//     */
//
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.995,
//            0.1,
//            0.1,
//            0.009,
//            50,
//            1.25,
//            10,
//            1
//    );
//
//    //Add custom localizers or drivetrains here
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .mecanumDrivetrain(driveConstants)
//                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .build();
//    }
//}