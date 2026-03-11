package org.firstinspires.ftc.teamcode.newpid;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@TeleOp
@Configurable
@Disabled
public class StandardPIDDriveTest extends OpMode {
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0.001;
    public static Pose startPose = new Pose(0, 0, 0);
    public static Pose endPose = new Pose(72, 0, 0);
    PIDController controller;
    TelemetryManager telemetryM;
    DcMotorEx BL, BR, FL, FR;
    Timer timer;
    Buffer buffer;
    PinpointLocalizer pinpoint;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        controller = new PIDController(kp, ki, kd);
        BL = hardwareMap.get(DcMotorEx.class, ConfigNames.BL);
        BR = hardwareMap.get(DcMotorEx.class, ConfigNames.BR);
        FL = hardwareMap.get(DcMotorEx.class, ConfigNames.FL);
        FR = hardwareMap.get(DcMotorEx.class, ConfigNames.FR);
        timer = new Timer(TimeUnit.MILLISECONDS);
        buffer = new Buffer(10, 0.01);
        pinpoint = new PinpointLocalizer(
                hardwareMap,
                ConstantsOldBot.pinpointLocalizerConstants,
                startPose
        );
    }

    @Override
    public void loop() {
        pinpoint.update();

        double loopTime = timer.getDeltaTime(TimeUnit.MILLISECONDS);
        Pose position = pinpoint.getPose();
        Pose velocity = pinpoint.getVelocity();
        double acceleration = buffer.getDerivative();
        double power = controller.calculate(endPose.getX() - position.getX());
        BL.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        FR.setPower(power);
        buffer.add(velocity.getX(), timer.getTime());

        telemetryM.addData("loop time", loopTime);
        telemetryM.addData("position", position);
        telemetryM.addData("velocity", velocity);
        telemetryM.addData("accel", acceleration);
        telemetryM.addData("target", endPose);
        telemetryM.addData("error", position.getX() - endPose.getX());

        telemetryM.update(telemetry);
    }
}
