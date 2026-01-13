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
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.readwrite.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
@Autonomous(name = "12CloseRightSort", group = "Competition")
public class TwelveBallCloseRightSort extends CommandOpMode {

    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;
    //TODO:FIX POSES
    public static Pose startPose = new Pose(118, 130, Math.toRadians(217));
    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(300));
    public static Pose shootPose = new Pose(87, 95, Math.toRadians(220));
    public static Pose parkPose = new Pose(85, 109, Math.toRadians(0));
    public static Pose openGatePose = new Pose(128, 69, Math.toRadians(0));
    public static Pose intakeOnePose = new Pose(100, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(100, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(100, 36, Math.toRadians(0));
    public static double xChangeIntake = 23;
    PathChain toMotifPath;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    MotifWriteCommand motifCommand = null;

    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;

    Command firstPath;

    //follower constraints
    public static double pathDistThresholdMin = 0.5;
    public static double headingError = 0.025;
    public static double timeOutConstraint = 200;
    public static double tValueConstraint = 0.97;

    //other misc
    public static int[] shootArray = new int[]{2, 1, 0};

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    ShootSeqCommand seqShootCommand;
    Follower follower;
    Timer gameTimer;
    public static double globalMaxPower = 0.9;

    Limelight3A limelight;
    Spindexer spindexer;
    TwoWheelShooter shooter;
    Intake intake;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    boolean prevVisionComplete = false;
    FtcDashboard dashboard;
    TelemetryPacket dashboardPacket;

    public static double maxTimeMs = 20500;
    public static double maxWritePoseTimeMs = 200;
    public static double maxSideWriteTimeMs = 200;

    boolean stopEnd = false;
    ShootSide side;
    boolean postMotif = false;
    boolean gameTimerStarted = false;
    public static boolean useColorSensors = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        super.reset();

        gameTimer = new Timer();

        initializeMechanisms();



        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setPose(startPose);

        dashboard = FtcDashboard.getInstance();
        dashboardPacket = new TelemetryPacket();

        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();

        buildStartPath();
        setupVision();
        if(motifDetection() != null) {
            schedule(motifDetection());
        }
    }
    @Override
    public void run(){
        super.run();


    }

    protected void buildStartPath(){
        toMotifPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifDetectionPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(pathDistThresholdMin)
                .setTValueConstraint(tValueConstraint)
                .build();
    }

    protected void initializeMechanisms() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap, useColorSensors).setBallColors(startBallColors).initAngle();
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
    }
    protected void setupVision(){
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
    }


    protected Command motifDetection(){
        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);
        firstPath = new FollowPathCommand(follower, toMotifPath, false).setGlobalMaxPower(globalMaxPower);
        return new ParallelCommandGroup(
                firstPath,
                motifCommand
        );
    }




}
