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
import org.firstinspires.ftc.teamcode.localization.camera.BallPather;
import org.firstinspires.ftc.teamcode.localization.camera.NormalPather;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp
public class AutoDriveToBallsTest extends CommandOpMode {
    TelemetryManager telemetryM;

    LimelightDetector limelightDetector;
    AutoDriveToBalls driveCommand;
    WheelControl2 drive;
    Follower follower;
    boolean started = false;

    @Override
    public void initialize() {
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        drive = new WheelControl2(hardwareMap);
        limelightDetector = new LimelightDetector(limelight, follower::getPose);
        BallPather ballPather = new NormalPather();
        driveCommand = new AutoDriveToBalls(follower, ballPather, limelightDetector, drive, 0.5);

        limelightDetector.start();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

//        Vector2d[] ballPixels = limelightDetector.getBallPixels();
//        Pose[] ballPoses = limelightDetector.getBallPoses(follower.getPose(), ballPixels);

//        telemetryM.addData("Ball pose length", ballPoses.length);
//        for (int i = 0; i < ballPoses.length; i++) {
//            telemetryM.addData("Ball " + i, ballPoses[i]);
//        }
//        for (int i = 0; i < ballPixels.length; i++) {
//            telemetryM.addLine("Ball " + i + " pixel x: " + ballPixels[i].getX());
//            telemetryM.addLine("Ball " + i + " pixel y: " + ballPixels[i].getY());
//        }
        if (!started) {
            driveCommand.schedule();
            started = true;
        }
        telemetryM.update(telemetry);

    }

    @Override
    public void end() {
        limelightDetector.close();
    }
}
