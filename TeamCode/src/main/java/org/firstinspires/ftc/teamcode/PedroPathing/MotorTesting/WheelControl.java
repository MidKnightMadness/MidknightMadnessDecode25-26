package org.firstinspires.ftc.teamcode.PedroPathing.MotorTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//
public class WheelControl {
    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;
    public double vf = 0;
    public double hf = 0;
    public double rf = 0;

    public WheelControl(HardwareMap hardwareMap) {
        this.BR = hardwareMap.get(DcMotorEx.class, "BR");
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FR = hardwareMap.get(DcMotorEx.class, "FR");
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.BL = hardwareMap.get(DcMotorEx.class, "BL");
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FL = hardwareMap.get(DcMotorEx.class, "FL");
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.BL.setDirection(DcMotorEx.Direction.REVERSE);
        this.FL.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setF(double vf, double hf, double rf) {
        this.vf = vf;
        this.hf = hf;
        this.rf = rf;
    }

    public void setPowers(double BL, double BR, double FL, double FR, double power) {
        double max = 1; // max motor power
        max = Math.max(BL, max);
        max = Math.max(BR, max);
        max = Math.max(FL, max);
        max = Math.max(FR, max); // Detect the motor with the most power
        this.BL.setPower(power * (BL/max));
        this.BR.setPower(power * (BR/max));
        this.FL.setPower(power * (FL/max));
        this.FR.setPower(power * (FR/max)); // We divide all values by the maximum one so they do not reach one.
    }

    public void drive_relative(double forward, double right, double rotate_power, double max_power) {
        /*
        Positive rotate_power is CCW, negative is CW

        Wheel drive directions
        back back
        back back

        Wheel diagonals
        \ /
        / \
         */

        // Make sure forward and right are <= 1
        double power_scale = max_power/Math.max(max_power, Math.max(forward, right));
        forward *= power_scale;
        right *= power_scale;

        // Add feedforwards
        forward += Math.signum(forward)*vf;
        right += Math.signum(right)*hf;
        rotate_power += Math.signum(rotate_power)*rf;

        // Calculate motor powers
        double BLPower = forward - right - rotate_power;
        double BRPower = forward + right + rotate_power;
        double FLPower = forward + right - rotate_power;
        double FRPower = forward - right + rotate_power;

        // Get max power to make sure powers <= max_power
        double new_max_power = 1;
        new_max_power = Math.max(Math.abs(BLPower), new_max_power);
        new_max_power = Math.max(Math.abs(BRPower), new_max_power);
        new_max_power = Math.max(Math.abs(FLPower), new_max_power);
        new_max_power = Math.max(Math.abs(FRPower), new_max_power);

        // Set powers
        this.BL.setPower(BLPower/new_max_power);
        this.BR.setPower(BRPower/new_max_power);
        this.FL.setPower(FLPower/new_max_power);
        this.FR.setPower(FRPower/new_max_power);
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