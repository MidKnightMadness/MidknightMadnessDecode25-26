package org.firstinspires.ftc.teamcode.pedroPathing.motorTesting;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

//
public class WheelControl2 {
    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;

    PIDController driveController = new PIDController(0.1, 0, 0.001);
    PIDController headingController = new PIDController(1.0, 0, 0.01);

    public WheelControl2(HardwareMap hardwareMap) {
        this.BR = hardwareMap.get(DcMotorEx.class, ConfigNames.BR);
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.FR = hardwareMap.get(DcMotorEx.class, ConfigNames.FR);
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.BL = hardwareMap.get(DcMotorEx.class, ConfigNames.BL);
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.FL = hardwareMap.get(DcMotorEx.class, ConfigNames.FL);
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public WheelControl2 setPidControllers(PIDController drive, PIDController heading) {
        this.driveController = drive;
        this.headingController = heading;
        return this;
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

    public void driveRelative(double forward, double right, double rotate_power, double max_power) {
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

        setPowers(BLPower, BRPower, FLPower, FRPower, max_power);
    }

    public void driveFieldCentric(double driveX, double driveY, double rotate, double maxPower, double robotHeadingRad, ShootSide shootSide) {

        //if right shoot side: forward = +x, left = +y, right = -y, back = -x
        //if left shoot side: forward = -x, left = -y, right = +y, back = +x

        double forward = driveY * Math.cos(robotHeadingRad) - driveX * Math.sin(robotHeadingRad);
        double right   = driveY * Math.sin(robotHeadingRad) + driveX * Math.cos(robotHeadingRad);

        if(shootSide == ShootSide.RIGHT){
            forward *= -1;
            right *= -1;
        }
        driveRelative(forward, right, rotate, maxPower);
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

    public void driveAngle(double strafeAngle, double rotatePower, double drivePower, double robotHeading) {
        double theta = strafeAngle - robotHeading;
        double forward = drivePower*Math.cos(theta);
        double right = -drivePower*Math.sin(theta);
        driveRelative(forward, right, rotatePower, 1);
    }

    public void pid(Pose robotPose, Pose target) {
        double drivePower = driveController.calculate(robotPose.distanceFrom(target));
        pid(robotPose, target, drivePower);
    }

    public void pid(Pose robotPose, Pose target, double targetPower) {
        double strafeAngle = target.minus(robotPose).getAsVector().getTheta();
        double headingPower = headingController.calculate(
                MathFunctions.normalizeAngleSigned(
                        robotPose.getHeading() - target.getHeading()
                )
        );
        driveAngle(strafeAngle, headingPower, targetPower, robotPose.getHeading());
    }

    public void pidNoHeading(Pose robotPose, Pose target) {
        double drivePower = driveController.calculate(robotPose.distanceFrom(target));
        pidNoHeading(robotPose, target, drivePower);
    }

    public void pidNoHeading(Pose robotPose, Pose target, double targetPower) {
        double strafeAngle = target.minus(robotPose).getAsVector().getTheta();
        driveAngle(strafeAngle, 0, targetPower, robotPose.getHeading());
    }

    public void changeMode(DcMotor.ZeroPowerBehavior mode){
        this.BR.setZeroPowerBehavior(mode);
        this.FR.setZeroPowerBehavior(mode);
        this.BL.setZeroPowerBehavior(mode);
        this.FL.setZeroPowerBehavior(mode);
    }
}