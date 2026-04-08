package org.firstinspires.ftc.teamcode.main.autonomous.intakeTest;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.Timer;


@Config
@Configurable
@Disabled
@Autonomous(name = "Intake Auto Test", group = "Competititon")
public class NonColorIntakeAutoTest extends CommandOpMode {
    public static long firstWaitTime = 300;//700
    public static long secondWaitTime = 200;//500
    public static long thirdWaitTime = 200;//500

    public static long fourthWaitTime = 500;
    public static double pathDistThresholdMin = 0.5;
    public static double headingError = 0.015;
    public static double timeOutConstraint = 200;
    public static double xChangeIntake = 27;
    Follower follower;
    Timer gameTimer;
    Pose startPose;

    Limelight3A limelight;
    Spindexer spindexer;
    TwoWheelShooter shooter;
    Intake intake;
    TelemetryManager telemetryManager;
    boolean prevVisionComplete = false;
    public static Pose leftTargetPose = new Pose(12, 132, 0);
    public static Pose rightTargetPose = new Pose(132, 132, 0);
    FtcDashboard dashboard;
    TelemetryPacket dashboardPacket;

    public static double goToSpot1Time = 500;
    public static double goToSpot2Time = 500;

    public static double maxTimeMs = 29500;
    public static double maxWritePoseTimeMs = 200;
    public static double maxSideWriteTimeMs = 200;
    public static double[] pidBotGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kBotGainsShooter = new double[]{0, 0.00005, 0};
    public static double[] pidTopGainsShooter = new double[]{0.0004, 0, 0.00001};
    public static double[] kTopGainsShooter = new double[]{0.02, 0.00005, 0};

    boolean stopEnd = false;
    ShootSide side;
    boolean postMotif = false;
    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
    public static Pose intakeOnePose = new Pose(100, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(100, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(100, 36, Math.toRadians(0));
    boolean started = false;

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
//        buildPaths();
//        setupVision();
//        if(preMotifSequence() != null) {
//            schedule(preMotifSequence());
//
//        }
    }

    private void initializeMechanisms() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap, false).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();

        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
//        shooter.setRunMode(TwoWheelShooter.RunMode.RawPower);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        shooter.low.setVeloCoefficients(pidBotGainsShooter[0], pidBotGainsShooter[1], pidBotGainsShooter[2]);
        shooter.high.setVeloCoefficients(pidTopGainsShooter[0], pidTopGainsShooter[1], pidTopGainsShooter[2]);
        shooter.low.setFeedforwardCoefficients(kBotGainsShooter[0], kBotGainsShooter[1], kBotGainsShooter[2]);
        shooter.high.setFeedforwardCoefficients(kTopGainsShooter[0], kTopGainsShooter[1], kTopGainsShooter[2]);

    }


    int currSpindexerSpot = 0;

    @Override
    public void run(){
        super.run();

        if(!started){
            schedule(intake(1));
            started = true;
        }
        spindexer.goToSpot(SpindexerSpot.fromIndex(currSpindexerSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);

        //   if(postMotifSequence().isFinished()){
//            if(goToIntakeLine()!= null){
//                schedule(goToIntakeLine());
//            }
        //    }
//        if (timer.getTime() >= maxTimeMs) requestOpModeStop();

        updateTelemetry();
    }


    protected Command intake(int targetSpot){
//        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, intakeTime);
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setDirectPower(1.0)),
                        new SequentialCommandGroup(
                                new WaitCommand(firstWaitTime),
                                new InstantCommand(() -> currSpindexerSpot = 1),
                                new WaitCommand(secondWaitTime),
                                new InstantCommand(() -> currSpindexerSpot = 2),
                                new WaitCommand(thirdWaitTime)
                        ),
                        driveToIntakeEnd(targetSpot)
                ),
                new InstantCommand(()-> intake.stopPower())
        );

//        return intakePower(milliSec);
    }


    protected SequentialCommandGroup driveToIntakeEnd(int spot){
        Pose intakePose = (spot == 1) ? intakeOnePose : (spot == 2) ? intakeTwoPose : intakeThreePose;

        follower.update();
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin)
                        .setMaxPower(1.0)
        );
    }

    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
//        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Curr Spot", currSpindexerSpot);
        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addData("Current Follower Pose", follower.getPose().getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());
//        telemetry.addData("Start Ball Color 0", startBallColors[0]);
//        telemetry.addData("Start Ball Color 1", startBallColors[1]);
//        telemetry.addData("Start Ball Color 2", startBallColors[2]);

        telemetry.addData("New Ball Detected", spindexer.newBallDetected());
//        if(seqShootCommand != null) {
//            telemetry.addData("Seq Test Farthest Moved", seqShootCommand.farthestMoved);
//        }

//        telemetry.addData("Motif", motifPattern);
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
//        if(spots != null) {
//            telemetry.addData("Spindexer Optimal Sequence 0", spots[0]);
//            telemetry.addData("Spindexer Optimal Sequence 1", spots[1]);
//            telemetry.addData("Spindexer Optimal Sequence 2", spots[2]);
//        }
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

        //Shooter
        if(shooter != null){
            telemetry.addData("Shooter Dir RunMode", shooter.runMode);
            telemetry.addData("Shooter RunMode", shooterRunMode);
            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
        }

        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        //Intake
        if(intake != null){
            telemetry.addData("Intake Power", intake.getRightMotor().getPower());
            telemetry.addData("Intake Velocity", intake.getMotorVelocity());
        }

        //Time
//        telemetry.addData("Auto Elapsed Time", currentTime);
//        telemetry.addData("Update Rate", 1 / gameTimer.getDeltaTime());


        telemetry.addData("Spindexer Get Curr Angle", spindexer.getCurrentAngle());
//

        telemetry.update();

    }



}
