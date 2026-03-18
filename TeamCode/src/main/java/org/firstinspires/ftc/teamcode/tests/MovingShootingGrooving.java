package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ExtraFns;

@TeleOp
public class MovingShootingGrooving extends CommandOpMode {
    double turnPower;
    double currSpeed;
    double targetHeading;
    double headingError;
    boolean autoAlign;
    Follower follower;
    WheelControl2 wheelControl;
    TwoWheelShooter.AimCalculator aimCalculator;
    ShootSide shootSide;
    PIDController pidAutoAlign = new PIDController(1.0, 0, 0.1);
    TelemetryManager telemetryM;

    @Override
    public void initialize() {
        autoAlign = false;
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        wheelControl = new WheelControl2(hardwareMap)
                .setFRDirection(DcMotorSimple.Direction.FORWARD)
                .setBRDirection(DcMotorSimple.Direction.FORWARD)
                .setFLDirection(DcMotorSimple.Direction.REVERSE)
                .setBLDirection(DcMotorSimple.Direction.REVERSE);
        shootSide = ShootSide.LEFT;
        currSpeed = 0.8;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        aimCalculator = new TwoWheelShooter.AimCalculator();
        follower.setPose(new Pose(0, 0, 0));
        follower.setStartingPose(new Pose(0, 0, 0));
    }


    @Override
    public void run() {
        follower.update();
        toggleAutoAlign();

        turnPower = calculateAlignTurnPower();
        if (!autoAlign) {
            wheelControl.driveRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, currSpeed);
        } else {
            wheelControl.driveRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, turnPower, currSpeed);
        }

        telemetryM.addData("Left stick y (forward)", gamepad1.left_stick_y);
        telemetryM.addData("Left stick x (strafe)", gamepad1.left_stick_x);
        telemetryM.addData("Right stick x (turn)", gamepad1.right_stick_x);
        telemetryM.addData("Pose X", follower.getPose().getX());
        telemetryM.addData("Pose Y", follower.getPose().getY());
        telemetryM.addData("Pose Heading", follower.getPose().getHeading());
        telemetryM.addData("Velocity X", follower.getVelocity().getXComponent());
        telemetryM.addData("Velocity Y", follower.getVelocity().getYComponent());
        telemetryM.addData("Target Heading", targetHeading);
        telemetryM.addData("Turn Power", turnPower);
        telemetryM.addData("Auto Align", autoAlign);
        telemetryM.update(telemetry);
    }

    public double getAngleError(double currentHeading, double targetHeading) {
        //heading is in absolute radians
        double error = targetHeading - currentHeading;
        error = ExtraFns.normAnglePlusMinusPI(error);
        return error;
    }

    public double calculateAlignTurnPower() {
        double[] aimData = aimCalculator.targetPowersHeading(
                follower.getPose(),
                follower.getVelocity(),
                TwoWheelShooter.getShootPose(shootSide)
        );
        telemetryM.addLine("version 100");
        telemetryM.addLine("aim data stuff" + aimData[0] + " " + aimData[1] + " " + aimData[2]);
        targetHeading = MathFunctions.normalizeAngle(aimData[2] + Math.PI);
        headingError = getAngleError(
                follower.getPose().getHeading(),
                targetHeading
        );
        return -pidAutoAlign.calculate(headingError);
    }

    private void toggleAutoAlign() {
        if (gamepad1.leftBumperWasPressed()) {
            autoAlign = !autoAlign;
        }
    }
}
