package org.firstinspires.ftc.teamcode.Localization.Pinpoint;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalization {
    // Create an instance of the sensor
    GoBildaPinpointDriver pinpoint;

    Pose2D initPose;
    Pose2D currPose;
    public PinpointLocalization(HardwareMap hardwareMap, Pose2D startingPos) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        pinpoint.setOffsets(-138.874, -33, DistanceUnit.MM);

        setPose(startingPos);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
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

    public Pose2D getCurrentPose(){
        return currPose;
    }

    public Pose2D update() {
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();
        return pose2D;
    }

}
