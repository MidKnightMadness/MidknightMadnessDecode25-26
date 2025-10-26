package org.firstinspires.ftc.teamcode.localization.Pinpoint;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Deprecated
@Configurable
@Config
public class PinpointLocalization {
    // Create an instance of the senso
    GoBildaPinpointDriver pinpoint;

    Pose2D initPose;
    Pose2D currPose;
    public static double xOffsetMultiplier = 1;
    public static double yOffsetMulitplier = 1;
    public static double xOffset = 138.874;
    public static double yOffset = 33;
    public static GoBildaPinpointDriver.EncoderDirection xPod = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPod = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public PinpointLocalization(HardwareMap hardwareMap, Pose2D startingPos) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        pinpoint.setOffsets(xOffset * xOffsetMultiplier, yOffset * yOffsetMulitplier, DistanceUnit.MM);//swapped so xy planar cartesian
        //using left odometry pod

        setPose(startingPos);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(xPod, yPod);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        initPose = startingPos;
    }

    public void setPose(Pose2D pose){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));
    }

    public void recalibrateIMU(){
        pinpoint.recalibrateIMU();
    }

    public double getPinpointHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
    public Pose2D getCurrentPose(){
        return currPose;
    }

    public Pose2D update() {
        pinpoint.update();

        currPose = new Pose2D(DistanceUnit.INCH, pinpoint.getPosX(DistanceUnit.INCH) + initPose.getX(DistanceUnit.INCH), -pinpoint.getPosY(DistanceUnit.INCH) + initPose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES));
        return currPose;
    }

}
