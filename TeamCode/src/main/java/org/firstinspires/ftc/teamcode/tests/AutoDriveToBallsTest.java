package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.commands.AutoDriveToBalls;
import org.firstinspires.ftc.teamcode.hardware.LimelightDetector;
import org.firstinspires.ftc.teamcode.localization.camera.Pather;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp
public class AutoDriveToBallsTest extends CommandOpMode {
    public static double fovX = 54.5, fovY = 42;
    public static int resX = 640, resY = 480;
    public static double pitch = 0, roll = 0;
    public static double z = 11.4;
    public static double drivePower = 0.5;

    TelemetryManager telemetryM;

    LimelightDetector limelightDetector;
    AutoDriveToBalls driveCommand;
    Follower follower;
    boolean started = false;

    @Override
    public void initialize() {
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        WheelControl2 drive = new WheelControl2(hardwareMap);
        limelightDetector = new LimelightDetector(limelight);
//        driveCommand = new AutoDriveToBalls(follower, pather, limelight, drive, drivePower);

        limelightDetector.start();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void run() {
        super.run();

        Vector2d[] ballPixels = limelightDetector.getBallPixels();
        Pose[] ballPoses = limelightDetector.getBallPoses(follower.getPose(), ballPixels);

        telemetryM.addData("Ball pose length", ballPoses.length);
        for (int i = 0; i < ballPoses.length; i++) {
            telemetryM.addData("Ball " + i, ballPoses[0]);
        }
        for (int i = 0; i < ballPixels.length; i++) {
            telemetryM.addLine("Ball " + i + " pixel x: " + ballPixels[i].getX());
            telemetryM.addLine("Ball " + i + " pixel y: " + ballPixels[i].getY());
        }
        telemetryM.update(telemetry);
//        telemetryM.addData("Ball pose 0", ballPoses[0]);
//        if (!started) {
//            driveCommand.schedule();
//            started = true;
//        }
    }

    @Override
    public void end() {
        limelightDetector.close();
    }
}
