package org.firstinspires.ftc.teamcode.main.autonomous.sort.right;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand2;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootUpdateCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.main.autonomous.IntakeLine;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.tests.opModes.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Map;

@Config
@Configurable
@Autonomous(name = "9 Far Right Sort", group = "Competition")
public class NineBackRightSort extends BaseAuto {
    int startPipeline = 1;
    public static Pose startPose = new Pose(88, 8, Math.toRadians(270));
    public static Pose shootPose = new Pose(84, 17, Math.toRadians(247));
    public static Pose forwardPose = new Pose(88, 12, Math.toRadians(90));
    public static Pose parkPose = new Pose(86, 38, Math.toRadians(0));
    public static Pose openGatePose = new Pose(136, 76, Math.toRadians(180));
    public static Pose intakeCloseStartPose = new Pose(100.5, 84, Math.toRadians(0));
    public static Pose intakeCloseEndPose = new Pose(125, 84, Math.toRadians(0));
    public static Pose intakeMidStartPose = new Pose(100.5, 58, Math.toRadians(0));
    public static Pose intakeMidEndPose= new Pose(125, 58, Math.toRadians(0));
    public static Pose intakeFarStartPose = new Pose(100.5, 36, Math.toRadians(0));
    public static Pose intakeFarEndPose = new Pose(132, 36, Math.toRadians(0));
    public static Pose intakeCornerStartPose = new Pose(130, 17, Math.toRadians(0));
    public static Pose intakeCornerEndPose = new Pose(130, 11, Math.toRadians(0));



    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose;
    public static long firstWaitTime = 700;
    public static long secondWaitTime = 500;
    public static long thirdWaitTime = 500;//250 old

    public static long fourthWaitTime = 500;

    public static double pathDistThresholdMin = 0;
    public static double headingError = 0;
    public static double timeOutConstraint = 0;
    public static double velConstraint = 0;

    //TODO: TRY VELOCITY CONSTRAINT
    public static TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.RawPower;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    public static TwoWheelShooter.ShootDist shootDist = TwoWheelShooter.ShootDist.Far;
    public static boolean voltageCompensation = false;//TODO:TRY FALSE
    public static boolean recoveryOn = true;
    public static boolean useLUT = false;
    public static boolean useBulkMode = true;
//    Path toShootPresets;
//    Path toIntakeLineFar;
//    Path toIntakeEndThree;
//    Path toShootOne;
//    Path toIntakeLineMid;
//    Path toIntakeEndTwo;
//    Path toShootTwo;
//    Path toIntakeLineClose;
//    Path toIntakeEndOne;
//    Path toShootThree;
//    Path toPark;

    AprilTagDetection tag21;
    AprilTagDetection tag22;
    AprilTagDetection tag23;
    int motifTag = 23;
    PushUpServo pushUpServo;

    public void useLeftConstants(){
        if(getShootSide() == ShootSide.LEFT) {
            startPose = applyLeft(startPose);
            shootPose = applyLeft(shootPose);
            forwardPose = applyLeft(forwardPose);
            parkPose = applyLeft(parkPose);
            openGatePose = applyLeft(openGatePose);
            intakeCloseStartPose = applyLeft(intakeCloseStartPose);
            intakeCloseEndPose = applyLeft(intakeCloseEndPose);
            intakeMidStartPose = applyLeft(intakeMidStartPose);
            intakeMidEndPose = applyLeft(intakeMidEndPose);
            intakeFarStartPose = applyLeft(intakeFarStartPose);
            intakeFarEndPose = applyLeft(intakeFarEndPose);
            intakeCornerStartPose = applyLeft(intakeCornerStartPose);
            intakeCornerEndPose = applyLeft(intakeCornerEndPose);
            shootSide = ShootSide.LEFT;
        }
    }

    public Pose applyLeft(Pose pose){
        return new Pose(144 - pose.getX(), pose.getY(), normAngle(Math.toRadians(Math.PI - pose.getHeading())));
    }

    public double normAngle(double angle){
        while(angle < 0){
            angle += Math.PI * 2;
        } while(angle > 2 * Math.PI){
            angle -= Math.PI * 2;
        }
        return angle;
    }


    public ShootSide getShootSide(){
        return shootSide;
    }

    @Override
    public Pose getStartPose(){
        return startPose;
    }

    @Override
    public void setupVision(){
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        limelight.pipelineSwitch(startPipeline);
        limelight.start();
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, ConfigNames.arducam);
        file = createFile(fileName, directoryName);

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            RobotLog.ee("Log", "Error instantiating file writer of file: " + e.getMessage());
        }

    }

    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

    String fileName = "motif_value.txt";
    String directoryName = "competition";
    FileWriter fileWriter;
    File file;
    boolean finishedWritingMotif = false;
    public static boolean useDistanceSensor = false;
    public static double inBetweenTime = 100;
    public static boolean rawPowerOn = false;
    public static long powerFlywheelTime = 3000;
    int aprilTagID = 0;
    public static double intakeDrivePower = 0.3;
    int currSpindexerGotoSpot = -1;
    public static double spindexerSpeed = -0.50;
    public static boolean useAutoIntake = false;
    public static double intakePower = 0.8;

    boolean velAgressiveComp = false;
    boolean shootOn;
    BallColor[] currSpindexerBallColors;
    int triggeredSpot = -1;
    boolean triggerBallShot = false;
    int recentTriggeredSpot = -1;
    boolean scheduledPark = false;
    public static double maxTimeSwap1 = 1000;
    public static double maxTimeSwap2 = 1000;
    @Override
    public void initialize_loop(){
//        LLResult result = limelight.getLatestResult();
//        if (result != null) {
//            List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
//            if(list.size() != 0) {
//                LLResultTypes.FiducialResult item = list.get(0);
//                aprilTagID = item.getFiducialId();
//                motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
//            }
//        }
        arducam.update();
        tag21 = arducam.getTagBySpecificId(21);
        tag22 = arducam.getTagBySpecificId(22);
        tag23 = arducam.getTagBySpecificId(23);
        if(tag21 != null){
            aprilTagID = 21;
            motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
        } else if(tag22 != null){
            aprilTagID = 22;
            motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
        } else if(tag23 != null){
            aprilTagID = 23;
            motifPattern = idMap.getOrDefault(aprilTagID, MotifEnums.Motif.NONE);
        }
    }

    @Override
    public void writeMotif(){
        if (motifPattern != MotifEnums.Motif.NONE) {
            writeToFile(fileWriter, String.valueOf(aprilTagID));
            closeFileWriter(fileWriter);
            finishedWritingMotif = true;
        }
    }


    private static File createFile(String fileName, String dirName){
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        if(!dir.exists()){
            dir.mkdirs();
        }
        File file = new File(dir, fileName);
        return file;
    }


    private void writeToFile(FileWriter fileWriter, String s){
        try {
            fileWriter.write(s);
            fileWriter.flush();
        } catch (IOException e) {
            RobotLog.ee("Log", "No file writer detected: " + e.getMessage());
        }
    }

    private void closeFileWriter(FileWriter fileWriter){
        try {
            fileWriter.close();
        } catch (IOException e) {
            RobotLog.ee("Log", "Cannot close file writer: " + e.getMessage());
        }
    }

    Map<Integer, MotifEnums.Motif> idMap = Map.of(
            21, MotifEnums.Motif.GPP,
            22, MotifEnums.Motif.PGP,
            23, MotifEnums.Motif.PPG
    );
    //keep these empty and build the path using follower's current Pose


    @Override
    protected void buildPaths(){
        useLeftConstants();
//        toShootPresets = new Path(new BezierLine(forwardPose, shootPose));
//        toShootPresets.setLinearHeadingInterpolation(forwardPose.getHeading(), shootPose.getHeading());
//
//        toIntakeLineFar = new Path(new BezierLine(shootPose, intakeFarStartPose));
//        toIntakeLineFar.setLinearHeadingInterpolation(shootPose.getHeading(), intakeFarStartPose.getHeading());
//        setConstraints(toIntakeLineFar);

//        toIntakeEndThree = new Path(new BezierLine(intakeThreePose, intakeThreeEnd));
//        toIntakeEndThree.setLinearHeadingInterpolation(intakeThreePose.getHeading(), intakeThreeEnd.getHeading());
////        setConstraints(toIntakeEndThree);

//        toShootOne = new Path(new BezierLine(intakeThreeEnd, shootPose));
//        toShootOne.setLinearHeadingInterpolation(intakeThreeEnd.getHeading(), shootPose.getHeading());
//        setConstraints(toShootOne);
//
//        toIntakeLineMid = new Path(new BezierLine(shootPose, intakeMidStartPose));
//        toIntakeLineMid.setLinearHeadingInterpolation(shootPose.getHeading(), intakeMidStartPose.getHeading());
//        setConstraints(toIntakeLineMid);

//        toIntakeEndTwo = new Path(new BezierLine(intakeTwoPose, intakeTwoEnd));
//        toIntakeEndTwo.setLinearHeadingInterpolation(intakeTwoPose.getHeading(), intakeTwoEnd.getHeading());
////        setConstraints(toIntakeEndTwo);

//        toShootTwo = new Path(new BezierLine(intakeTwoEnd, shootPose));
//        toShootTwo.setLinearHeadingInterpolation(intakeTwoEnd.getHeading(), shootPose.getHeading());
////        setConstraints(toShootTwo);
//
//        toIntakeLineClose = new Path(new BezierLine(shootPose, intakeCloseStartPose));
//        toIntakeLineClose.setLinearHeadingInterpolation(shootPose.getHeading(), intakeCloseStartPose.getHeading());
//        setConstraints(toIntakeLineClose);

//   /     setConstraints(toIntakeEndOne);

//        toShootThree = new Path(new BezierLine(intakeOneEnd, shootPose));
//        toShootThree.setLinearHeadingInterpolation(intakeOneEnd.getHeading(), shootPose.getHeading());
//        setConstraints(toShootThree);

//        toPark = new Path(new BezierLine(shootPose, parkPose));
//        toPark.setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading());
//        setConstraints(toPark);

    }


    private void setConstraints(Path path){
        if(timeOutConstraint != 0) {
            path.setTimeoutConstraint(timeOutConstraint);
        }
        if(pathDistThresholdMin != 0) {
            path.setTranslationalConstraint(pathDistThresholdMin);
        }
        if(headingError != 0) {
            path.setHeadingConstraint(headingError);
        }
        if(velConstraint != 0){
            path.setVelocityConstraint(velConstraint);
        }
    }


    @Override
    protected boolean isVisionComplete(){
        return true;
    }


     @Override
    protected Command preMotifSequence(){
        return null;
    }

    @Override
    public void update(){
         //override to park if not enough time
         if(!scheduledPark && gameTimer.getTime() >= 27000){
             scheduledPark = true;
             CommandScheduler.getInstance().cancelAll();
             schedule(new ParallelCommandGroup(
                     new InstantCommand(()-> currSpindexerGotoSpot = 0),
                     park()
             ));
         }

        if(currSpindexerGotoSpot != -1) {
            spindexer.goToSpot(SpindexerSpot.fromIndex(currSpindexerGotoSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }



        if(velAgressiveComp && !shooter.inRecoveryMode){
            velAgressiveComp = false;
        }


        if(shooter.low.getVelocity() > 200  || shooter.low.motor.getPower() > 0.1){
            shootOn = true;
        } else{
            shootOn = false;
        }
        spindexer.updateShootOn(shootOn);


        if(!shootOn || currSpindexerBallColors == null){
            return;
        }


        if(recoveryOn) {
            if (!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT0, SpotType.OUTTAKE)) {
                triggeredSpot = 0;
                recentTriggeredSpot = triggeredSpot;
                velAgressiveComp = true;
            } else if (!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT1, SpotType.OUTTAKE)) {
                triggeredSpot = 1;
                recentTriggeredSpot = triggeredSpot;
                velAgressiveComp = true;
            } else if (!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT2, SpotType.OUTTAKE)) {
                triggeredSpot = 2;
                recentTriggeredSpot = triggeredSpot;
                velAgressiveComp = true;
            } else {
                triggeredSpot = -1;
                triggerBallShot = false;
            }


            if (!triggerBallShot && triggeredSpot != -1) {
                shooter.triggerBallShot(recoveryOn);
                spindexer.removeBall(triggeredSpot);
                triggerBallShot = true;
            }
        }
    }

    @Override
    protected void initializeMechanisms() {
        spindexer = new Spindexer(hardwareMap, useDistanceSensor).setBallColors(startBallColors).initAngle();
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        pushUpServo = new PushUpServo(hardwareMap);
        register(intake, shooter, spindexer, pushUpServo);
        if(useBulkMode) {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        } else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }
    }




    @Override
    protected Command postMotifSequence(){
//        limelight.stop();
//        limelight.shutdown();
        arducam.stop();
        //temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
//                driveForward(),
                setSpindexerCorrect(IntakeLine.CLOSE),//GPP so that G is on the right side
                new WaitCommand(1000),
                shoot(),

                getToLineNum(IntakeLine.FAR),
                intake(IntakeLine.FAR),
                shoot(),

                getToLineNum(IntakeLine.CORNER),
                intake(IntakeLine.CORNER),
                shoot(),

                getToLineNum(IntakeLine.MID),
                intake(IntakeLine.MID),
                shoot(),

                new ParallelCommandGroup(
                    new InstantCommand(()-> currSpindexerGotoSpot = 0),
                    park()
                )
        );
    }

    protected Command setSpindexerCorrect(IntakeLine lineNum){
        if(lineNum == IntakeLine.CLOSE) {
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 2),
                            () -> motifPattern == MotifEnums.Motif.PGP
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        } else if(lineNum == IntakeLine.FAR){
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        }  else if(lineNum == IntakeLine.MID) {
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PGP
                    ),
                    () -> motifPattern == MotifEnums.Motif.PPG
            );
        }
        else{//lineNum == IntakeLIne.Corner which is the same as mid
            return new ConditionalCommand(
                    new InstantCommand(),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PGP
                    ),
                    () -> motifPattern == MotifEnums.Motif.PPG
            );
        }
    }
    protected Command shoot(){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                getToShootCommand(),
                                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                                new InstantCommand(() -> spindexer.getTurner().setRunMode(CRServoEx2.RunMode.RawPower)),
//                                new InstantCommand(() -> spindexer.getTurner2().setRunMode(CRServoEx2.RunMode.RawPower)),
                                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed)),
                                new WaitCommand(powerFlywheelTime)
                        ),
                        new InstantCommand(()-> pushUpServo.setUp()),
                        new ShootUpdateCommand(spindexer, shooter, follower, shootSide, useLUT, voltageCompensation, shootDist, rawPowerOn)
                ),
                new ParallelCommandGroup(
                    new InstantCommand(() -> currSpindexerGotoSpot = 0),
                    new InstantCommand(() -> pushUpServo.setDown())
                )
        );
    }

    protected SequentialCommandGroup setDefaultStartColors(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.setBallColors(startBallColors))
        );
    }


    protected Command intake(IntakeLine lineNum){

        Pose linePose = lineNum == IntakeLine.FAR ? intakeFarStartPose : lineNum == IntakeLine.MID ? intakeMidStartPose : lineNum == IntakeLine.CLOSE ? intakeCloseStartPose : intakeCornerStartPose;

         if(!useAutoIntake) {
             return new SequentialCommandGroup(
                     new InstantCommand(() -> intake.setDirectPower(1.0)),
                     new ParallelCommandGroup(
                             //new AutoIntakeCommand(spindexer, intake, 1.0, 20000, inBetweenTime),
                             new SequentialCommandGroup(
                                     //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(0), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
                                     new WaitCommand(firstWaitTime),
                                     new InstantCommand(() -> currSpindexerGotoSpot = 1),
                                     //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(1), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
                                     new WaitCommand(secondWaitTime),
                                     new InstantCommand(() -> currSpindexerGotoSpot = 2),
                                     //new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(2), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl, 500),
                                     new WaitCommand(thirdWaitTime)
                             ),
                             driveToIntakeEnd(lineNum)
                     ).withTimeout(4500),
                     new ParallelCommandGroup(
                             new InstantCommand(() -> intake.setDirectPower(0)),
                             setSpindexerCorrect(lineNum)
                     )
             );
         } else{
             return new SequentialCommandGroup(
                     new ParallelRaceGroup(
                         new SequentialCommandGroup(
                             new SchedulePathTo(follower, linePose).setMaxPower(1.0),
                             new ParallelCommandGroup(
                                 driveToIntakeEnd(lineNum)
                             ).withTimeout(4500)
                         ),
                         new AutoIntakeCommand2(spindexer, intake, intakePower, inBetweenTime, useDistanceSensor)
                    ),
                 setSpindexerCorrect(lineNum)
             );
         }
    }

    protected SchedulePathTo driveToIntakeEnd(IntakeLine lineNum){
        Pose endPose = lineNum == IntakeLine.CLOSE ? intakeCloseEndPose : lineNum == IntakeLine.MID ? intakeMidEndPose : lineNum == IntakeLine.FAR ? intakeFarEndPose : intakeCornerEndPose;
//        if(spot == 3) {
//            return new SchedulePathTo(follower, intakeFarEndPose).setMaxPower(intakeDrivePower);
//        } else if(spot == 2){
//            return new SchedulePathTo(follower, intakeMidEndPose).setMaxPower(intakeDrivePower);
//        } else {
//            return new SchedulePathTo(follower, intakeCloseEndPose).setMaxPower(intakeDrivePower);
//        }
        return new SchedulePathTo(follower, endPose).setMaxPower(intakeDrivePower);
    }

    protected Command park(){
        return new SchedulePathTo(follower, parkPose);
    }

    protected Command getToLineNum(IntakeLine lineNum){
//        FollowPathCommand command;
//        if(lineNum == IntakeLine.FAR){
//            command = new FollowPathCommand(follower, toIntakeLineFar);
//        }
//        else if(lineNum == IntakeLine) {
//            command = new FollowPathCommand(follower, toIntakeLineMid);
//        }
//        else {
//            command = new FollowPathCommand(follower, toIntakeLineClose);
//        }

        Pose linePose = lineNum == IntakeLine.FAR ? intakeFarStartPose : lineNum == IntakeLine.MID ? intakeMidStartPose : lineNum == IntakeLine.CLOSE ? intakeCloseStartPose : intakeCornerStartPose;
        return new ParallelDeadlineGroup(
                new SchedulePathTo(follower, linePose).setMaxPower(1.0),
                new AutoIntakeCommand2(spindexer, intake, intakePower, inBetweenTime, useDistanceSensor, maxTimeSwap1, maxTimeSwap2)
        );
    }

    protected SchedulePathTo getToShootCommand(){
        return new SchedulePathTo(follower, shootPose).setMaxPower(1.0);
    }


    @Override
    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
        currentPose = follower.getPose();
//        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addData("Curr Spindexer GotoSpot", currSpindexerGotoSpot);
        telemetry.addData("Current Voltage", shooter.getCurrVoltage());
        telemetry.addData("Ratio Voltage ", shooter.getTargetVoltage() / shooter.getCurrVoltage());

        telemetry.addData("Current Follower Pose", currentPose.getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());
        telemetry.addData("Start Ball Color 0", startBallColors[0]);
        telemetry.addData("Start Ball Color 1", startBallColors[1]);
        telemetry.addData("Start Ball Color 2", startBallColors[2]);

//        telemetry.addData("New Ball Detected", spindexer.newBallDetected());

        telemetry.addData("Motif", motifPattern);
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
//        telemetry.addData("All Occupied", spindexer.allOccuppiedBallColors());
//
        //Shooter
        if(shooter != null){

            telemetry.addData("Shooter Low Vel", shooter.low.getVelocity());
            telemetry.addData("Shooter High Vel", shooter.high.getVelocity());
            telemetry.addData("Shooter Low Vel", shooter.low.getCorrectedVelocity());
            telemetry.addData("Shooter High Vel", shooter.high.getCorrectedVelocity());
            telemetry.addData("Shooter Dir RunMode", shooter.runMode);
            telemetry.addData("Shooter RunMode", shooterRunMode);
            telemetry.addData("Shooter Low RunMode", shooter.low.motorEx.getMode());
            telemetry.addData("Shooter High RunMode", shooter.low.motorEx.getMode());
        }

        //Spindexer
        if(spindexer != null){
            telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle().toDegrees());
//            telemetry.addData("Balls Left", spindexer.getBallCount());
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }




//

        telemetry.update();
//        graphManager.update();
//        telemetryManager.update();
//        if(dashboard!= null) {
//            dashboard.sendTelemetryPacket(dashboardPacket);
//        }
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
