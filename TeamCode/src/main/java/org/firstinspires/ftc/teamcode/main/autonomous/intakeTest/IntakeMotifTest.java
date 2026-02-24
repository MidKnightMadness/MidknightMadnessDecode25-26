package org.firstinspires.ftc.teamcode.main.autonomous.intakeTest;



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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandMotif;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPosition;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPositionSmooth;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;


@Config
@Configurable
@Autonomous(name = "Intake Motif Test")
public class IntakeMotifTest extends CommandOpMode {
    public static double pathDistThresholdMin = 1.5;
    public static double headingError = Math.toRadians(2);
    public static double timeOutConstraint = 100;
    Follower follower;
    Timer gameTimer;
    SpindexerNonCR spindexer;
    Intake intake;

    public static double inBetweenTime = 0;


    public static Pose intakeCloseStartPose = new Pose(97, 84, Math.toRadians(0));
    public static Pose intakeCloseEndPose = new Pose(125, 84, Math.toRadians(0));
    public static Pose intakeMidStartPose = new Pose(97, 56, Math.toRadians(0));
    public static Pose intakeMidEndPose = new Pose(129, 56, Math.toRadians(0));
    public static Pose intakeFarStartPose = new Pose(97, 34, Math.toRadians(0));
    public static Pose intakeFarEndPose = new Pose(135, 34, Math.toRadians(0));
    public static double intakePower = 1.0;
    AutoIntakeCommandMotif autoIntakeCommand;
    public static MotifEnums.Motif targetMotif = MotifEnums.Motif.PGP;
    public static MotifEnums.Motif intakeOrder = MotifEnums.Motif.GPP;
    public static double drivePower = 0.3;
    public static int targetSpot = 1;
    boolean started;
    Path path;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
        super.reset();

        gameTimer = new Timer();
        gameTimer.restart();

        initializeMechanisms();

        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setPose(intakeFarStartPose);


        path = new Path(new BezierLine(intakeFarStartPose, intakeFarEndPose));
        path.setLinearHeadingInterpolation(intakeFarStartPose.getHeading(), intakeFarEndPose.getHeading());
    }

    private void setConstraints(Path path){
        path.setTimeoutConstraint(timeOutConstraint);
        path.setTranslationalConstraint(pathDistThresholdMin);
        path.setHeadingConstraint(headingError);
    }
    private void initializeMechanisms() {
        spindexer = new SpindexerNonCR(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
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
        autoIntakeCommand = new AutoIntakeCommandMotif(spindexer, intake, intakePower, inBetweenTime, targetMotif, intakeOrder, hardwareMap);
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    driveToIntakeEnd(),
                    autoIntakeCommand.withTimeout(10000)
                ),
                new SpindexerGotoPositionSmooth(spindexer, SpindexerSpotNonCR.fromIndex(3).getIntakePositionSolo(), 1.0)
        );
    }


    protected FollowPathCommand driveToIntakeEnd(){
        return new FollowPathCommand(follower, path, true).setGlobalMaxPower(drivePower);
    }

    protected void updateTelemetry(){
        telemetry.addData("Intake Order", intakeOrder);
        telemetry.addData("Target Motif", intakeOrder);
        if(autoIntakeCommand!= null && autoIntakeCommand.isScheduled()){
            telemetry.addData("Intake Sequence",  autoIntakeCommand.getSpotsSeq());
            telemetry.addData("Curr Spot(0-3)",  autoIntakeCommand.getCurrSpot());
            telemetry.addData("Curr Index(0-2)",  autoIntakeCommand.getCurrentIndex());
        }
        telemetry.addData("Follower Pose", follower.getPose());

        telemetry.update();
    }



}
