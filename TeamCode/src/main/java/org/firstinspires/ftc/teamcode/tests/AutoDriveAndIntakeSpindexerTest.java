package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.commands.AutoDriveToBalls;
import org.firstinspires.ftc.teamcode.commands.AutoDriveToBallsSpindexer;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandTime;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.hardware.LimelightDetector;
import org.firstinspires.ftc.teamcode.localization.camera.BallPather;
import org.firstinspires.ftc.teamcode.localization.camera.NormalPather;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp
public class AutoDriveAndIntakeSpindexerTest extends CommandOpMode {
    TelemetryManager telemetryM;

    LimelightDetector limelightDetector;
    Command driveAndIntakeCommand;
    WheelControl2 drive;
    Follower follower;
    boolean started = false;
    BallPather ballPather;
    SpindexerNonCR spindexer;
    Intake intake;

    @Override
    public void initialize() {
        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        drive = new WheelControl2(hardwareMap);
        ballPather = new NormalPather();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        limelightDetector = new LimelightDetector(limelight);

        spindexer = new SpindexerNonCR(hardwareMap,
                true,
                new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        driveAndIntakeCommand = new ParallelDeadlineGroup(
                new AutoIntakeCommandTime(
                        spindexer,
                        intake,
                        1.0,
                        false,
                        hardwareMap.voltageSensor.iterator().next(),
                        0,
                        1,
                        100
                ),
                new AutoDriveToBallsSpindexer(
                        follower,
                        ballPather,
                        limelightDetector,
                        drive,
                        spindexer,
                        1500,
                        0.5
                )
        );

        limelightDetector.start();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        if (!started) {
            driveAndIntakeCommand.schedule();
            started = true;
        }
        telemetryM.update(telemetry);

    }

    @Override
    public void end() {
        limelightDetector.close();
    }
}
