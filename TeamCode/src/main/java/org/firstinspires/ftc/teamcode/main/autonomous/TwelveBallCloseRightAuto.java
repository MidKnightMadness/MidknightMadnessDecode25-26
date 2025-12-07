package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;


import org.firstinspires.ftc.teamcode.commands.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeTimeCommand;
import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.commands.TurnToCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.lang.reflect.WildcardType;

@Config
@Configurable
@Autonomous(name = "12 Back Left", group = "Competition")
public class TwelveBallCloseRightAuto extends BaseAuto {
    public static double motifDetectionTimeMs = 3000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(118, 130, Math.toRadians(45));
    public static Pose motifDetectionPose = new Pose(87, 94, Math.toRadians(100));
    public static Pose shootPose = new Pose(87, 94, Math.toRadians(230));
    public static Pose parkPose = new Pose(114, 94, Math.toRadians(210));
    public static Pose openGatePose = new Pose(136, 76, Math.toRadians(180));
    public static Pose intakeOnePose = new Pose(110, 84, Math.toRadians(0));
    public static Pose intakeTwoPose = new Pose(110, 60, Math.toRadians(0));
    public static Pose intakeThreePose = new Pose(110, 36, Math.toRadians(0));
    public static double intakeDistForward = 14;
    PathChain toMotifPath;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.GPP;
    MotifWriteCommand motifCommand = null;

    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;

    Command firstPath;

    public static double pathDistThresholdMin = 0.5;
    public static double headingError = 0.025;
    public static double timeOutConstraint = 200;
    public static double intakeTime = 6000;
    public static double intakePower = 1;
    public static double xChangeIntake = 20;
    public static int[] shootArray = new int[]{2, 1, 0};

    public static BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] seq = SpindexerSpot.convertFromindex(new int[]{0, 0, 0});
    long intakeHelpTime = 3000;

    ShootSeqCommand seqTestCommand;
    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void setupVision(){
//        limelight.pipelineSwitch(startPipeline);
//        limelight.start();
    }



    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

    //keep these empty and build the path using follower's current Pose
    @Override
    protected void buildPaths(){
        toMotifPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, motifDetectionPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), motifDetectionPose.getHeading())
                .setHeadingConstraint(headingError)
                .setTimeoutConstraint(timeOutConstraint)
                .setTranslationalConstraint(pathDistThresholdMin)
                .setTValueConstraint(0.97)
                .build();
    }


    @Override
    public BallColor[] getStartBallColors(){
        return startBallColors;
    }

    @Override
    protected boolean isVisionComplete(){
//        motifPattern = motifCommand.getDetected();
//        if(motifCommand.isFinished()){
//            ConstantsBot.motifIsBusy = false;
//            return true;
//        }
//        if(timer.getTime() > timeOutConstraint){
//            firstPath.end(true);
//        }
        return firstPath.isFinished();
    }
    public static long waitTime = 500;
    AutoIntakeCommand autoIntakeCommand;
    boolean autoStart = false;

    @Override
    protected Command preMotifSequence(){
//        motifCommand = new MotifWriteCommand(limelight, motifDetectionTimeMs);

        firstPath = new FollowPathCommand(follower, toMotifPath, true).setGlobalMaxPower(0.9);
        return new SequentialCommandGroup(
                setDefaultStartColors(),
                firstPath
//                motifCommand
        );

    }
    @Override
    public void update(){

    }
    @Override

    protected void initializeMechanisms() {
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        if(getStartBallColors() != null){
            spindexer = new Spindexer(hardwareMap, true).setBallColors(getStartBallColors());
        }
        else{
            spindexer = new Spindexer(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE});
        }

        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);

        shooter.low.setVeloCoefficients(pidBotGainsShooter[0], pidBotGainsShooter[1], pidBotGainsShooter[2]);
        shooter.high.setVeloCoefficients(pidTopGainsShooter[0], pidTopGainsShooter[1], pidTopGainsShooter[2]);
        shooter.low.setFeedforwardCoefficients(kBotGainsShooter[0], kBotGainsShooter[1], kBotGainsShooter[2]);
        shooter.high.setFeedforwardCoefficients(kTopGainsShooter[0], kTopGainsShooter[1], kTopGainsShooter[2]);

        shooter.low.resetEncoder();
        shooter.high.resetEncoder();

    }

    @Override
    protected Command postMotifSequence(){
//        limelight.stop();//temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
//              getToShootCommand(1000),
                new WaitCommand(waitTime),
                getToShootCommand(500),
//                startShoot()
                new ParallelCommandGroup(
                shootOptimal()),
//                intakePower(intakeHelpTime)),
//
              getToLineNum(1, 500),
              new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 1000),
              new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
              intake(1, 5000),

              new ParallelCommandGroup(
                   new InstantCommand(() -> spindexer.goToSpot(SpindexerSpot.SPOT0, SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl)),
                   openGate(1000)
              ),

              getToShootCommand(500),
//              shoot(),
//
              getToLineNum(2, 500),
//              intake(1),
              getToShootCommand(500),
//              shoot(),
//
              getToLineNum(3, 500),
//              intake(3),
              getToShootCommand(500),
//              shoot(),
//
              park(100)
//                new InstantCommand(() -> spindexer.goTo(Angle.fromDegrees(0), CRServoEx2.RunMode.OptimizedPositionalControl))
//
        );
//        else{
//        return null;

    }


    private Command intakePowerByTime(long milliSec) {
        return new IntakeTimeCommand(intake, milliSec);
    }

    protected SequentialCommandGroup goToMotifDetection(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, motifDetectionPose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }
    protected SequentialCommandGroup setDefaultStartColors(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.setBallColors(startBallColors))
        );
    }
    protected SequentialCommandGroup openGate(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, openGatePose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }


    protected Command intake(int spot, long milliSec){
        autoIntakeCommand = new AutoIntakeCommand(spindexer, intake, intakePower, intakeTime);
        autoStart = true;
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    autoIntakeCommand,
                    driveToIntakeEnd(spot)
                ),
                new WaitCommand(milliSec)
        );
//        return intakePower(milliSec);
    }
    protected SequentialCommandGroup driveToIntakeEnd(int spot){
        Pose intakePose = (spot == 1) ? intakeOnePose : (spot == 2) ? intakeTwoPose : intakeThreePose;

        follower.update();
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, new Pose(intakePose.getX() + xChangeIntake, intakePose.getY(), intakePose.getHeading()), headingError, timeOutConstraint, pathDistThresholdMin));
    }

    protected SequentialCommandGroup park(long milliSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, parkPose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(milliSec)
        );
    }
    protected SequentialCommandGroup startShoot(){
        return new SequentialCommandGroup(
                new ShootSeqCommand(spindexer, shooter, SpindexerSpot.convertFromindex(shootArray), follower, shootSide, false, TwoWheelShooter.ShootDist.Close)
        );
    }
    protected SequentialCommandGroup shootOptimal(){
        seq = spindexer.getOptimalSequence(motifPattern);
        seqTestCommand = new ShootSeqCommand(spindexer, shooter, seq, follower, shootSide, false, TwoWheelShooter.ShootDist.Close);
        return new SequentialCommandGroup(
                seqTestCommand,
                new InstantCommand(() -> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}))
        );
    }
    protected SequentialCommandGroup getToLineNum(int lineNum, long milliSec){
        SchedulePathTo command = null;
        if(lineNum == 1) command = new SchedulePathTo(follower, intakeOnePose, headingError, timeOutConstraint, pathDistThresholdMin);
        else if(lineNum == 2) command =  new SchedulePathTo(follower, intakeTwoPose, headingError, timeOutConstraint, pathDistThresholdMin);
        else command = new SchedulePathTo(follower, intakeThreePose, headingError, timeOutConstraint, pathDistThresholdMin);

        return new SequentialCommandGroup(
                command,
                new WaitCommand(milliSec)
        );
    }

    protected SequentialCommandGroup getToShootCommand(long millSec){
        return new SequentialCommandGroup(
                new SchedulePathTo(follower, shootPose, headingError, timeOutConstraint, pathDistThresholdMin),
                new WaitCommand(millSec)
        );
    }


    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
        currentPose = follower.getPose();
        double currentTime = timer.getTime();

        // Follower
        telemetry.addData("Current Follower Pose", currentPose.getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());
        telemetry.addData("Start Ball Color 0", startBallColors[0]);
        telemetry.addData("Start Ball Color 1", startBallColors[1]);
        telemetry.addData("Start Ball Color 2", startBallColors[2]);

        telemetry.addData("New Ball Detected", spindexer.newBallDetected());
        if(seqTestCommand != null) {
            telemetry.addData("Seq Test Farthest Moved", seqTestCommand.farthestMoved);
        }

        telemetry.addData("Motif", motifPattern);
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
            telemetry.addData("Spindexer Optimal Sequence 0", seq[0]);
            telemetry.addData("Spindexer Optimal Sequence 1", seq[1]);
            telemetry.addData("Spindexer Optimal Sequence 1", seq[2]);
        }
        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());

        addToTelemGraph("Pose X", currentPose.getX());
        addToTelemGraph("Pose Y", currentPose.getY());
        addToTelemGraph("Pose Heading", currentPose.getHeading());
        addToTelemGraph("Follower T Value", follower.getCurrentTValue());
        addToTelemGraph("Follower Velocity X", follower.getVelocity().getXComponent());
        addToTelemGraph("Follower Velocity Y", follower.getVelocity().getYComponent());
        addToTelemGraph("Follower Velocity Mag", follower.getVelocity().getMagnitude());
        addToAllTelemGraph("Follower Velocity Heading", follower.getVelocity().getTheta());
        addToAllTelemGraph("Follower Translational Error", follower.getDriveError());
        addToAllTelemGraph("Follower Heading Error", follower.getHeadingError());
        addToTelemGraph("Follower Max Vel Constraint", follower.getConstraints().getVelocityConstraint());
        addToTelemGraph("Follower T Constraint", follower.getConstraints().getTValueConstraint());

        //Shooter
        if(shooter != null){
            addToAllTelemGraph("Shooter Low Flywheel Power", shooter.low.get());
            addToAllTelemGraph("Shooter High Flywheel Power", shooter.high.get());
            addToAllTelemGraph("Shooter Low Flywheel Vel", shooter.low.getVelocity());
            addToAllTelemGraph("Shooter High Flywheel Vel", shooter.high.getVelocity());
        }

        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        //Intake
        if(intake != null){
            addToAllTelemGraph("Intake Power", intake.getMotor().get());
            addToAllTelemGraph("Intake Velocity", intake.getMotorVelocity());
        }

        //Time
        addToAllTelemGraph("Auto Elapsed Time", currentTime);
        addToAllTelemGraph("Update Rate", 1 / timer.getDeltaTime());

        //Motif
        addBooleanToTelem("Motif Busy", !isVisionComplete());
        if(motifCommand != null){
            addStringToTelem("Motif Timer", String.valueOf(motifCommand.getTime()));
        }
        addStringToTelem("Motif Pattern", String.valueOf(motifPattern));

        //First Path
        if(firstPath != null){
            addBooleanToTelem("First Path Busy", !firstPath.isFinished());
        }

        telemetry.update();
        graphManager.update();
        telemetryManager.update();
        if(dashboard!= null) {
            dashboard.sendTelemetryPacket(dashboardPacket);
        }
    }



    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }
    public void addToAllTelemGraph(String s, Number o){
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
        telemetry.addData(s, o);
        if(dashboard != null) {
            dashboardPacket.put(s, o);
        };
    }
    public void addBooleanToTelem(String s, boolean o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
    }




}

