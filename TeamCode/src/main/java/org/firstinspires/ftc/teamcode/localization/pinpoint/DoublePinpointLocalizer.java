package org.firstinspires.ftc.teamcode.localization.pinpoint;

import android.annotation.SuppressLint;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import java.util.ArrayList;
import java.util.Objects;

//uses two pinpoints and averages the x, y, heading. Two odometry pods parallel and one split between both pinpoints
public class DoublePinpointLocalizer implements Localizer {

    PinpointLocalizer pinpoint1Localizer;
    PinpointLocalizer pinpoint2Localizer;
    private Pose startPose;
    private Pose currentPose;
    private Pose currentVelocity;
    private double previousHeading;
    private double totalHeading;
    public DoublePinpointLocalizer(HardwareMap map, PinpointConstants constants1, PinpointConstants constants2){ this(map, constants1, constants2, new Pose());}

    /**
     * This creates a new PinpointLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    @SuppressLint("NewApi")
    public DoublePinpointLocalizer(HardwareMap map, PinpointConstants constants1, PinpointConstants constants2, Pose setStartPose){
            pinpoint1Localizer = new PinpointLocalizer(map, constants1);
            pinpoint2Localizer = new PinpointLocalizer(map, constants2);
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return currentPose;
    }

    public static double wrap0to2PI(double angle) {
        while (angle < 0) angle += 2*Math.PI;
        while (angle > 2 * Math.PI) angle -= 2*Math.PI;
        return angle;
    }


    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }


    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }


    @Override
    public void setStartPose(Pose setStart) {
        pinpoint1Localizer.setStartPose(setStart);
        pinpoint2Localizer.setStartPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        pinpoint1Localizer.setPose(setPose);
        pinpoint2Localizer.setPose(setPose);
    }


    @Override
    public void update() {
        pinpoint1Localizer.update();
        pinpoint2Localizer.update();
        Pose localizer1Pose = pinpoint1Localizer.getPose();
        Pose localizer2Pose = pinpoint2Localizer.getPose();
        currentPose = new Pose((localizer1Pose.getX() + localizer2Pose.getX()) / 2,
                (localizer1Pose.getY() + localizer2Pose.getY()) / 2,
                (wrap0to2PI(localizer1Pose.getHeading() + localizer2Pose.getHeading()/2))
        );

        Pose currVel1 = pinpoint1Localizer.getVelocity();
        Pose currVel2 = pinpoint2Localizer.getVelocity();
        currentVelocity = new Pose((currVel1.getX() + currVel2.getX()) / 2,
                                    (currVel1.getY() + currVel2.getY()) / 2,
                                       (currVel1.getHeading() + currVel2.getHeading()) / 2
        );


        totalHeading += (pinpoint1Localizer.getTotalHeading() + pinpoint2Localizer.getTotalHeading()) / 2;
        previousHeading = currentPose.getHeading();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
     * @return returns the Y encoder value
     */
    @Override
    public double getForwardMultiplier() {
        return (double) (pinpoint1Localizer.getPinpoint().getEncoderX() + pinpoint2Localizer.getPinpoint().getEncoderX()) / 2;
    }

    /**
     * This returns the X encoder value as none of the odometry tuners are required for this localizer
     * @return returns the X encoder value
     */
    @Override
    public double getLateralMultiplier() {
        return (double) (pinpoint1Localizer.getPinpoint().getEncoderY() + pinpoint2Localizer.getPinpoint().getEncoderY()) / 2;
    }

    /**
     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
     * @return returns the yaw scalar
     */
    @Override
    public double getTurningMultiplier() {
        return (double) (pinpoint1Localizer.getPinpoint().getYawScalar() + pinpoint2Localizer.getPinpoint().getYawScalar()) / 2;
    }


    /**
     * This resets the IMU. Does not change heading estimation.
     */
    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    /**
     * This resets the pinpoint.
     */
    private void resetPinpoint() {
        pinpoint1Localizer.getPinpoint().resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pinpoint2Localizer.getPinpoint().resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This recalibrates the Pinpoint. It will take 0.25 seconds to recalibrate, and the robot must be still
     */
    public void recalibrate() {
        pinpoint1Localizer.recalibrate();
        pinpoint2Localizer.recalibrate();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }

    /**
     * This returns the GoBildaPinpointDriver object used by this localizer, in case you want to
     * access any of its methods directly.
     *
     * @return returns the GoBildaPinpointDriver object used by this localizer
     */
    public GoBildaPinpointDriver[] getPinpoints() {
        return new GoBildaPinpointDriver[]{pinpoint1Localizer.getPinpoint(), pinpoint2Localizer.getPinpoint()};
    }
}
