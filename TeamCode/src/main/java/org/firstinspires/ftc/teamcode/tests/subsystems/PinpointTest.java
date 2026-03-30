package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;

@TeleOp
public class PinpointTest extends OpMode {
    TelemetryManager telemetryM;
    WheelControl2 wheelControl;
    Follower follower;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = ConstantsOldBot.createPinpointFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 7, 0));
        wheelControl = new WheelControl2(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        wheelControl.pid(follower.getPose(), new Pose(30, 7, 0));
//        wheelControl.driveRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
        telemetryM.addData("Pose X", follower.getPose().getX());
        telemetryM.addData("Pose Y", follower.getPose().getY());
        telemetryM.addData("Pose heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.update(telemetry);
    }
}
