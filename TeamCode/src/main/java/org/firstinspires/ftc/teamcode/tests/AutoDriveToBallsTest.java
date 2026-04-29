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
    BallPather ballPather;
    Pose[] path;

    @Override
    public void initialize() {
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        drive = new WheelControl2(hardwareMap);
        limelightDetector = new LimelightDetector(limelight);
        ballPather = new NormalPather();
        driveCommand = new AutoDriveToBalls(follower, ballPather, limelightDetector, drive, 0.5);

        limelightDetector.start();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

//        Pose robotPose = follower.getPose();
//        if (path == null || path.length == 0) {
//            Vector2d[] ballPixels = limelightDetector.getBallPixels();
//            Pose[] ballPoses = limelightDetector.getBallPoses(follower.getPose(), ballPixels);
//            path = ballPather.findPath(follower.getPose(), ballPoses, 3);
////            started = true;
//        } else {
//            Pose target = path[0].withHeading(path[0].minus(robotPose).getAsVector().getTheta());
//            follower.holdPoint(target);
//        }

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
