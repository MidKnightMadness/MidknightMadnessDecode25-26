package org.firstinspires.ftc.teamcode.main.autonomous;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;


@Config
@Configurable
@Autonomous(name = "Color Intake Auto Test", group = "Competition")
public class ColorIntakeAutoTest extends CommandOpMode {
    public static double pathDistThresholdMin = 1.5;
    public static double headingError = Math.toRadians(2);
    public static double timeOutConstraint = 100;
    Follower follower;
    Timer gameTimer;
    Pose startPose;

    Limelight3A limelight;
    Spindexer spindexer;
    TwoWheelShooter shooter;
    Intake intake;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    boolean prevVisionComplete = false;
    public static Pose leftTargetPose = new Pose(12, 132, 0);
    public static Pose rightTargetPose = new Pose(132, 132, 0);
    FtcDashboard dashboard;
    TelemetryPacket dashboardPacket;

    public static double maxTimeMs = 29500;
    public static double waitTime = 200;
    public static double maxWritePoseTimeMs = 200;
    public static double maxSideWriteTimeMs = 200;

    public static Pose intakeOnePose = new Pose(102, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(102, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(102, 36, Math.toRadians(0));
    public static Pose intakeOneEndPose = new Pose(125, 84, Math.toRadians(0));
    public static Pose intakeTwoEndPose = new Pose(125, 60, Math.toRadians(0));
    public static Pose intakeThreeEndPose = new Pose(125, 36, Math.toRadians(0));
    boolean started = false;
    public static double intakePower = 1.0;
    AutoIntakeCommand autoIntakeCommand;
    public static double drivePower = 0.3;

    public static boolean useColor = false;
    public static boolean useDistance = true;
    public static int targetSpot = 1;
    Path path;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        super.reset();

        gameTimer = new Timer();
        gameTimer.restart();

        initializeMechanisms();

        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setPose(intakeOnePose);

        dashboard = FtcDashboard.getInstance();
        dashboardPacket = new TelemetryPacket();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();

        path = new Path(new BezierLine(intakeOnePose, intakeOneEndPose));
        path.setLinearHeadingInterpolation(intakeOnePose.getHeading(), intakeOneEndPose.getHeading());
        setConstraints(path);
    }

    private void setConstraints(Path path){
        path.setTimeoutConstraint(timeOutConstraint);
        path.setTranslationalConstraint(pathDistThresholdMin);
        path.setHeadingConstraint(headingError);
    }
    private void initializeMechanisms() {
        spindexer = new Spindexer(hardwareMap, useDistance).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        register(spindexer, intake);
    }

    @Override
    public void run(){
        super.run();

        follower.update();
        if(!started){
            schedule(intake());
            started = true;
        }


        updateTelemetry();
    }


    protected Command intake(){
        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, 8000, waitTime);
        return new ParallelCommandGroup(
                driveToIntakeEnd(),
                autoIntakeCommand.withTimeout(5000)
        );
    }


    protected FollowPathCommand driveToIntakeEnd(){
        return new FollowPathCommand(follower, path, true).setGlobalMaxPower(drivePower);
    }

    protected void updateTelemetry(){
        // Update pose & follower

        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Curr Spot", autoIntakeCommand.currNumBall);
        telemetry.addData("Current Follower Pose", follower.getPose().getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());

        telemetry.addData("New Ball Detected", spindexer.newBallDetected());

        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());

        telemetry.addData("Pose X", follower.getPose().getX());
        telemetry.addData("Pose Y", follower.getPose().getY());
        telemetry.addData("Pose Heading", follower.getPose().getHeading());
        telemetry.addData("Follower T Value", follower.getCurrentTValue());
        telemetry.addData("Follower Velocity X", follower.getVelocity().getXComponent());
        telemetry.addData("Follower Velocity Y", follower.getVelocity().getYComponent());
        telemetry.addData("Follower Velocity Mag", follower.getVelocity().getMagnitude());
        telemetry.addData("Follower Velocity Heading", follower.getVelocity().getTheta());
        telemetry.addData("Follower Translational Error", follower.getDriveError());
        telemetry.addData("Follower Heading Error", follower.getHeadingError());
        telemetry.addData("Follower Max Vel Constraint", follower.getConstraints().getVelocityConstraint());
        telemetry.addData("Follower T Constraint", follower.getConstraints().getTValueConstraint());


        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        //Intake
        if(intake != null){
            telemetry.addData("Intake Power", intake.getMotor().get());
            telemetry.addData("Intake Velocity", intake.getMotorVelocity());
        }

        //Time
        telemetry.addData("Auto Elapsed Time", currentTime);
        telemetry.addData("Update Rate", 1 / gameTimer.getDeltaTime());


        telemetry.addData("Spindexer Get Curr Angle", spindexer.getCurrentAngle());
//

        telemetry.update();

    }



}
