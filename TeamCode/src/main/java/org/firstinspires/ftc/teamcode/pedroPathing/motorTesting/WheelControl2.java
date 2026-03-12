package org.firstinspires.ftc.teamcode.pedroPathing.motorTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

//
public class WheelControl2 {
    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;

    public WheelControl2(HardwareMap hardwareMap) {
        this.BR = hardwareMap.get(DcMotorEx.class, ConfigNames.BR);
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FR = hardwareMap.get(DcMotorEx.class, ConfigNames.FR);
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BL = hardwareMap.get(DcMotorEx.class, ConfigNames.BL);
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FL = hardwareMap.get(DcMotorEx.class, ConfigNames.FL);
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPowers(double BL, double BR, double FL, double FR, double power) {
        double max = 1; // max motor power
        max = Math.max(Math.abs(BL), max);
        max = Math.max(Math.abs(BR), max);
        max = Math.max(Math.abs(FL), max);
        max = Math.max(Math.abs(FR), max); // Detect the motor with the most power
        this.BL.setPower(power * (BL/max));
        this.BR.setPower(power * (BR/max));
        this.FL.setPower(power * (FL/max));
        this.FR.setPower(power * (FR/max)); // We divide all values by the maximum one so they do not reach one.
    }

    public void drive_relative(double forward, double right, double rotate_power, double max_power) {
        /*3z
        Positive rotate_power is CCW, negative is CW

        Wheel drive directions
        back back
        back back

        Wheel diagonals
        \ /
        / \
         */

        // Make sure forward and right are <= 1

        double power_scale = max_power / Math.max(
                max_power, Math.max(Math.abs(forward), Math.abs(right))
        );
        double rotate_scale = max_power / Math.max(max_power, Math.abs(rotate_power));

        forward *= power_scale;
        right *= power_scale;
        rotate_power *= rotate_scale;

        // Calculate motor powers
        double BLPower = forward - right + rotate_power;
        double BRPower = forward + right - rotate_power;
        double FLPower = forward + right + rotate_power;
        double FRPower = forward - right - rotate_power;

        setPowers(BLPower, BRPower, FLPower, FRPower, 1);
    }

    public void driveFieldCentric(double driveX, double driveY, double rotate, double maxPower, double robotHeadingRad, ShootSide shootSide) {

        //if right shoot side: forward = +x, left = +y, right = -y, back = -x
        //if left shoot side: forward = -x, left = -y, right = +y, back = +x

        double forward = driveY * Math.cos(robotHeadingRad) - driveX * Math.sin(robotHeadingRad);
        double right   = driveY * Math.sin(robotHeadingRad) + driveX * Math.cos(robotHeadingRad);

        forward *= -1;//bc gamepad reversed for some reason
        if(shootSide == ShootSide.RIGHT){
            forward *= -1;
            right *= -1;
        }
        drive_relative(forward, right, rotate, maxPower);
    }

    public WheelControl2 setBLDirection(DcMotorSimple.Direction direction) {
        this.BL.setDirection(direction);
        return this;
    }

    public WheelControl2 setBRDirection(DcMotorSimple.Direction direction) {
        this.BR.setDirection(direction);
        return this;
    }

    public WheelControl2 setFLDirection(DcMotorSimple.Direction direction) {
        this.FL.setDirection(direction);
        return this;
    }

    public WheelControl2 setFRDirection(DcMotorSimple.Direction direction) {
        this.FR.setDirection(direction);
        return this;
    }

    public void stop() {
        this.BL.setPower(0);
        this.BR.setPower(0);
        this.FL.setPower(0);
        this.FR.setPower(0);
    }

    public void drive_angle(double strafe_angle, double rotate_power, double drive_power, double robot_heading) {
        /*
        Everything is in degrees
        Strafe angle is relative to field, not robot
        Power only comes from variable and not forward/right
        Useful if you want to drive at an exact power
        Rotation is added after drive
        Angles are standard (0 is positive x-axis, 90 is positive y-axis)
        Useful for driving given power and angle (polar)
        */

        // Turn strafe angle heading clockwise
        double theta = Math.toRadians(strafe_angle-robot_heading);

        // Convert angle and power to relative drive
        double forward = drive_power*Math.cos(theta);
        double right = -drive_power*Math.sin(theta);

        // Drive relatively
        drive_relative(forward, right, rotate_power, 1);
    }

    public void drive_limit_power(double drive_x, double drive_y, double rotate_power, double max_drive_power, double robot_heading) {
        /*
        Everything is in degrees
        Similar to drive but power is used as max
        Rotation is added after drive
        Angles are standard (0 is positive x-axis, 90 is positive y-axis)
        Useful for driving given x and y powers (rectangular)
         */

        // Convert x and y relative to robot forward and right
        robot_heading = Math.toRadians(robot_heading);
        double forward = drive_x*Math.cos(robot_heading) + drive_y*Math.sin(robot_heading);
        double right = drive_x*Math.sin(robot_heading) - drive_y*Math.cos(robot_heading);

        // Drive relatively
        drive_relative(forward, right, rotate_power, max_drive_power);
    }

    public void change_mode(DcMotor.ZeroPowerBehavior mode){
        this.BR.setZeroPowerBehavior(mode);
        this.FR.setZeroPowerBehavior(mode);
        this.BL.setZeroPowerBehavior(mode);
        this.FL.setZeroPowerBehavior(mode);
    }
}