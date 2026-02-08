package org.firstinspires.ftc.teamcode.main.autonomous.sort.leftCopies;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.WaitUntilShootReadyCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand2;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand3;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.readwrite.PoseWriteCommand;
import org.firstinspires.ftc.teamcode.commands.readwrite.SideWriteCommand;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.main.autonomous.BuildPath;
import org.firstinspires.ftc.teamcode.main.autonomous.IntakeLine;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.tests.camera.CamCommand;
import org.firstinspires.ftc.teamcode.tests.opModes.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

@Config
@Configurable
@Autonomous(name = "Mayhem BL Cam Slow Intake", group = "Competition")
public class MayhemAutoCamBRSlowIntake extends BaseAuto {
    int objectDetectionPipeline = 3;
    public static Pose startPose = new Pose(86, 8.8, Math.toRadians(270));
    public static Pose shootPose = new Pose(83, 17, Math.toRadians(249));
    //    public static double shootOffset = Math.toRadians(2);
//public static Pose shootPose = new Pose(84, 17, Math.toRadians(247));
    public static Pose parkPose = new Pose(86, 38, Math.toRadians(0));
    public static Pose openGatePose = new Pose(136, 76, Math.toRadians(180));
    public static Pose intakeCloseStartPose = new Pose(97, 84, Math.toRadians(0));
    public static Pose intakeCloseEndPose = new Pose(125, 84, Math.toRadians(0));
    public static Pose intakeMidStartPose = new Pose(97, 5658, Math.toRadians(0));
    public static Pose intakeMidEndPose = new Pose(129, 56, Math.toRadians(0));

    public static Pose intakeFarStartPose = new Pose(97, 34, Math.toRadians(0));
    public static Pose intakeFarEndPose = new Pose(135, 34, Math.toRadians(0));
    public static Pose intakeCornerStartPose = new Pose(121, 12, Math.toRadians(0));
    public static Pose intakeCornerEndPose = new Pose(129, 12, Math.toRadians(0));
    public static Pose intakeCornerStartPose2 = new Pose(123, 6, Math.toRadians(0));
    public static Pose intakeCornerEndPose2 = new Pose(129, 6, Math.toRadians(0));
    public static Pose closeShootPose = new Pose(91.2, 84.6, Math.toRadians(230));

    public static Pose startDetectPose = new Pose(110, 10, Math.toRadians(0));
    public static Pose strafeFourPose = new Pose(110, 50, Math.toRadians(0));


    public static long driveIntakeEndTime = 5000;


    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
   ShootSide shootSide = ShootSide.LEFT;
    Pose currentPose = startPose;
    public static long firstWaitTime = 700;
    public static long secondWaitTime = 500;
    public static long thirdWaitTime = 500;//250 old

    public static long fourthWaitTime = 500;

    public static double pathDistThresholdMin = 0;
    public static double headingError = 0;
    public static double timeOutConstraint = 0;
    public static double velConstraint = 0;

    //TODO: TRY VELOCITY CONSTRAINT
    public static TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    SpindexerSpot[] spots;

    public static TwoWheelShooter.ShootDist shootDist = TwoWheelShooter.ShootDist.Far;
    public static boolean voltageCompensation = false;//TODO:TRY FALSE
    public static boolean recoveryOn = true;
    public static boolean useLUT = false;
    public static boolean useBulkMode = true;
    PathChain toShootPresets;
    PathChain toIntakeLineFarStart;
    PathChain toIntakeLineFarEnd;
    PathChain toIntakeLineMidStart;
    PathChain toIntakeLineMidEnd;
    PathChain toIntakeLineCornerEnd;
    PathChain toIntakeLineCornerStart;
    PathChain toShootFromFar;
    PathChain toShootFromMid;
    PathChain toShootFromClose;
    PathChain toShootFromCorner;
    PathChain toIntakeLineCloseStart;
    PathChain toIntakeLineCloseEnd;
    PathChain toIntakeLineCornerBack;
    PathChain toIntakeLineCornerEnd2;
    PathChain toShootCloseFromMid;

    PathChain toShootCloseFromClose;

    PathChain toShootCloseFromFar;

    PathChain toPark;

    public static double strafePower = 0.5;
    AprilTagDetection tag21;
    AprilTagDetection tag22;
    AprilTagDetection tag23;
    int motifTag = 23;
    PushUpServo pushUpServo;
    int limelightPipelineStart = 3;
    boolean isReadyToShoot;

    public void useLeftConstants(){
        if(shootSide== ShootSide.LEFT) {
//            startPose = applyLeft(startPose);
            shootPose = new Pose(144- shootPose.getX(), shootPose.getY(), normAngle(Math.PI - Math.toRadians(245)));
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
            intakeCornerStartPose2 = applyLeft(intakeCornerStartPose2);
            intakeCornerEndPose2 = applyLeft(intakeCornerEndPose2);
            closeShootPose = applyLeft(closeShootPose);
            startDetectPose = applyLeft(startDetectPose);
            strafeFourPose = applyLeft(strafeFourPose);
            shootSide = ShootSide.LEFT;
            targetXFirst = 144 - targetXFirst;
            targetXEnd = 144 - targetXEnd;
        }
    }

    public Pose applyLeft(Pose pose){
        return new Pose(144 - pose.getX(), pose.getY(), normAngle((Math.PI - pose.getHeading())));
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
        if(shootSide == ShootSide.LEFT){
            return applyLeft(startPose);
        } return startPose;
    }

    @Override
    public void setupVision(){
//        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
//        limelight.pipelineSwitch(objectDetectionPipeline);
//        limelight.start();

        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, ConfigNames.arducam);
        file = createFile(fileName, directoryName);

        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(limelightPipelineStart);
        limelight.start();

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
    boolean useDistanceSensor = true;
    public static double inBetweenTime = 200;
    public static boolean rawPowerOn = false;
    public static long powerFlywheelTime = 1200;
    int aprilTagID = 0;
    BuildPath buildPath;
    public static long timeoutCorner = 100;
    public static double intakeDrivePower = 0.3;
    public static double intakeCornerDrivePower = 0.6;
    int currSpindexerGotoSpot = -1;
    public static double spindexerSpeed = 0.7;
    public static double intakePower = 0.8;
    public static double autoCameraDrivePower = 0.6;
    boolean velAgressiveComp = false;
    boolean shootOn;
    BallColor[] currSpindexerBallColors;
    int triggeredSpot = -1;
    boolean triggerBallShot = false;
    int recentTriggeredSpot = -1;
    boolean scheduledPark = false;
    public static double maxTimeSwap1 = 1000;
    public static double maxTimeSwap2 = 1000;
    public static long maxWaitTillShoot = 3000;
    public static double targetXFirst = 128;
    public static double targetXEnd = 134;
    public static double cornerIntakePower = 1.0;
    public static double lowFlywheelTol = 40;
    public static double highFlywheelTol = 40;
    PathChain driveToStartPath;
    PathChain strafe1Path;
    boolean camDetect = false;
    public static long firstPresetWaitTime = 1400;
    public static long cornerTimeout = 1800;
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
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.update();
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

        toShootPresets = buildPath(startPose, shootPose);

        toIntakeLineFarStart = buildPath(shootPose, intakeFarStartPose);
        toIntakeLineFarEnd = buildPath(intakeFarStartPose, intakeFarEndPose);

        toShootFromFar = buildPath(intakeFarEndPose, shootPose);

        toIntakeLineCornerStart = buildPath(shootPose, intakeCornerStartPose);
        toIntakeLineCornerEnd = buildPathCorner(intakeCornerStartPose, intakeCornerEndPose);
        toIntakeLineCornerBack = buildPathCorner(intakeCornerEndPose, intakeCornerStartPose2);
        toIntakeLineCornerEnd2 = buildPathCorner(intakeCornerStartPose2, intakeCornerEndPose2);

        toShootFromCorner = buildPath(intakeCornerEndPose2, shootPose);

        toIntakeLineMidStart = buildPath(shootPose, intakeMidStartPose);

        toIntakeLineMidEnd = buildPath(intakeMidStartPose, intakeMidEndPose);

        toShootFromMid = buildPath(intakeMidEndPose, shootPose);

        toIntakeLineCloseStart = buildPath(shootPose, intakeCloseStartPose);
//
        toIntakeLineCloseEnd = buildPath(intakeCloseStartPose, intakeCloseEndPose);

        toShootFromClose = buildPath(intakeCloseEndPose, closeShootPose);
        toShootCloseFromFar = buildPath(intakeFarEndPose, closeShootPose);
        toShootCloseFromMid = buildPath(intakeMidEndPose, closeShootPose);
        toShootCloseFromClose = buildPath(intakeFarEndPose, closeShootPose);

        toPark = buildPath(shootPose, parkPose);

        driveToStartPath = buildPath(shootPose, startDetectPose);
        strafe1Path = buildPath(startDetectPose, strafeFourPose);
    }

    private PathChain buildPath(Pose startPose, Pose endPose){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        return path;
    }
    private PathChain buildPathCorner(Pose startPose, Pose endPose){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .setTimeoutConstraint(timeoutCorner)
                .build();
        return path;
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

    boolean continueShoot = false;
    double currVolt;
    boolean autoIntakeOn;
    AutoIntakeCommand2 autoIntakeCommand;
    int autoIntakeNum = -1;
    CamCommand cam;
    @Override
    public void update(){
        //override to park if not enough time
//         if(gameTimer.getTime() >= 27000 && !scheduledPark){
//             scheduledPark = true;
//             CommandScheduler.getInstance().cancelAll();
//             schedule(new ParallelCommandGroup(
//                     new InstantCommand(()-> currSpindexerGotoSpot = 0),
//                     park()
//             ));
//         }
//        if(autoIntakeOn && autoIntakeCommand != null){
//            currSpindexerGotoSpot = autoIntakeCommand.getSpotCurrent();
//        }
        if(currSpindexerGotoSpot != -1) {
            spindexer.goToSpot(SpindexerSpot.fromIndex(currSpindexerGotoSpot), SpotType.INTAKE, CRServoEx2.RunMode.OptimizedPositionalControl);
        }




        if(continueShoot){
            currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
            shooter.setFlywheelStaticPresets(shootDist, voltageCompensation, currVolt);
        } else{
            shooter.stopFlywheels();
        }

        if(autoIntakeCommand != null && autoIntakeOn){
            autoIntakeNum = autoIntakeCommand.getSpotCurrent();
        } else{
            autoIntakeNum = -1;
        }

//        if(shooter.readyToShoot()){
//            isReadyToShoot = true;
//        }

//        if(velAgressiveComp && !shooter.inRecoveryMode){
//            velAgressiveComp = false;
//        }
//
//        if(pushUpColor == GobildaLightBlock.Color.GREEN){
//            shootOn = true;
//        } else{
//            shootOn = false;
//        }
//
//        spindexer.updateShootOn(shootOn);
//
//        if(!shootOn || currSpindexerBallColors == null){
//            return;
//        }
//
//
//
//        if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT0, SpotType.OUTTAKE)){
//            triggeredSpot = 0;
//            velAgressiveComp = true;
//        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT1, SpotType.OUTTAKE)){
//            triggeredSpot = 1;
//            velAgressiveComp = true;
//        } else if(!velAgressiveComp && spindexer.isAtSpot(SpindexerSpot.SPOT2, SpotType.OUTTAKE)){
//            triggeredSpot = 2;
//            velAgressiveComp = true;
//        } else{
//            triggeredSpot = -1;
//            triggerBallShot = false;
//        }
//
//        if(!triggerBallShot && triggeredSpot != -1){
//            recentTriggeredSpot = triggeredSpot;
//            shooter.triggerBallShot(recoveryOn);
//            spindexer.removeBall(triggeredSpot);
//            triggerBallShot = true;
//        }
    }

    @Override
    protected void initializeMechanisms() {
        spindexer = new Spindexer(hardwareMap, useDistanceSensor).setBallColors(startBallColors).initAngle();
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        pushUpServo = new PushUpServo(hardwareMap);
        if(useBulkMode) {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
        } else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }
        register(intake, shooter, spindexer, pushUpServo);
    }




    @Override
    protected Command postMotifSequence(){
//        limelight.stop();
//        limelight.shutdown();
        arducam.stop();
        //temporarily turn it off to hand to localizer
        return new SequentialCommandGroup(
//                driveForward(),
                //GPP so that G is on the right side
                shootPreset(maxWaitTillShoot),

                intakeCorner(),
                shootFromLines(IntakeLine.CORNER, maxWaitTillShoot),

                intake(IntakeLine.FAR),
                shootFromLines(IntakeLine.FAR, maxWaitTillShoot),

//                intakeLast(),
//                shootFromLines(IntakeLine.CORNER, maxWaitTillShoot),
//
//                intake(IntakeLine.MID),
//                shootFromLines(IntakeLine.MID, maxWaitTillShoot)

                cameraWork(),

                new ParallelCommandGroup(
                        park(),
                        new InstantCommand(()-> currSpindexerGotoSpot = 0)
                )
        );
    }

    public Command driveToStartViewPosition(){
        return new FollowPathCommand(follower, driveToStartPath, true, 1.0);
    }

    protected Command cameraWork(){
        cam = new CamCommand(limelight, follower, shootSide);
        return new SequentialCommandGroup(
                driveToStartViewPosition(),
                new WaitCommand(200),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                cam,
                                new InstantCommand(()-> camDetect = true)
                        ),
                        new SequentialCommandGroup(
                                strafeNumber(1)
//                            new WaitCommand(waitBetweenStrafe),
////                            strafeNumber(2),
//                            new WaitCommand(waitBetweenStrafe),
//                            strafeNumber(3),
//                            new WaitCommand(waitBetweenStrafe),
//                            strafeNumber(4)
                        )
                ),
                intakeBall(),
                shootFromCam(maxWaitTillShoot)
        );
    }
    private Command strafeNumber(int numb){//start at 1
        PathChain pathChain = strafe1Path;
        return new FollowPathCommand(follower, pathChain, true, strafePower);
    }
    PathChain cameraForwardPathChain;
    public Command intakeBall(){
        buildPath = new BuildPath(follower, cam, targetXFirst, targetXEnd, shootPose);
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(()-> spindexer.setDefault()),
                new InstantCommand(()-> autoIntakeOn = true),
//                new WaitCommand(1000),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
                new ParallelRaceGroup(
                        new AutoIntakeCommand3(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap),
                        new SequentialCommandGroup(
                                buildPath,
                                new DeferredCommand(()-> new InstantCommand(()-> follower.update()), null),
                                new DeferredCommand(()-> new ConditionalCommand(
//                                    new InstantCommand(()-> cameraForwardPathChain = buildPath.getPathChain()),
//                                            new DeferredCommand(()->
//                                                new SequentialCommandGroup(
//                                                new InstantCommand(()-> cameraForwardPathChain = follower.pathBuilder()
//                                                .addPath(new BezierLine(follower.getPose(), new Pose(targetX, follower.getPose().getY(), follower.getPose().getHeading())))
//                                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), follower.getPose().getHeading()).build()),
//                                                new WaitCommand(500)
//                                            ), null),
                                        new InstantCommand(()-> cameraForwardPathChain = buildPath.getPathChain()),
                                        new SequentialCommandGroup(
                                                new InstantCommand(()-> CommandScheduler.getInstance().cancelAll()),
                                                new InstantCommand(()-> follower.breakFollowing()),
                                                new InstantCommand(()-> currSpindexerGotoSpot = 0),
                                                new WaitCommand(1000),
                                                new ParallelCommandGroup(
                                                        new PoseWriteCommand(follower.getPose(), maxWritePoseTimeMs),
                                                        new SideWriteCommand(getSide(), maxSideWriteTimeMs)
                                                ),
                                                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
                                                new WaitCommand(500),
                                                new InstantCommand(()-> requestOpModeStop())
                                        ),
                                        () -> buildPath.pathCreated), null),
                                new DeferredCommand(() -> new FollowPathCommand(follower, cameraForwardPathChain, true, autoCameraDrivePower), null),
                                new WaitCommand(500),
                                new InstantCommand(()-> cameraForwardPathChain = buildPath.getPathChain2()),
                                new DeferredCommand(() -> new FollowPathCommand(follower, cameraForwardPathChain, true, autoCameraDrivePower), null),
                                new WaitCommand(1000)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0)),
                new InstantCommand(()-> autoIntakeOn = false)
//                new InstantCommand(()-> currSpindexerGotoSpot = 2)
        );

//         }
    }

    protected Command setSpindexerCorrect(IntakeLine lineNum){
        if(lineNum == IntakeLine.CLOSE) {
            return new ConditionalCommand(
                    new InstantCommand(() -> currSpindexerGotoSpot = 0),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            new InstantCommand(() -> currSpindexerGotoSpot = 2),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        } else if(lineNum == IntakeLine.FAR){
            return new ConditionalCommand(
                    new InstantCommand(() -> currSpindexerGotoSpot = 1),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 2),
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        }  else {//middle & corner
            return new ConditionalCommand(
                    new InstantCommand(() -> currSpindexerGotoSpot = 2),
                    new ConditionalCommand(
                            new InstantCommand(() -> currSpindexerGotoSpot = 0),
                            new InstantCommand(() -> currSpindexerGotoSpot = 1),
                            () -> motifPattern == MotifEnums.Motif.PPG
                    ),
                    () -> motifPattern == MotifEnums.Motif.GPP
            );
        }
    }
    protected FollowPathCommand getToShootCloseCommand(IntakeLine intakeLine){
        PathChain path = intakeLine == IntakeLine.FAR ? toShootCloseFromFar : intakeLine == IntakeLine.MID ? toShootCloseFromMid : toShootCloseFromClose;
        return new FollowPathCommand(follower, path, true, 1.0);
    }
    protected Command shootClose(IntakeLine lineNum, long waitTime){
        return new SequentialCommandGroup(
                new InstantCommand(()-> autoIntakeOn = false),
                new InstantCommand(()-> intake.setDirectPower(0)),
                new ParallelCommandGroup(
                        getToShootCloseCommand(lineNum),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                setSpindexerCorrect(lineNum),
                                new WaitCommand(500),
                                new InstantCommand(() -> continueShoot = true),
                                new InstantCommand(() -> shootDist = TwoWheelShooter.ShootDist.Close),
                                new InstantCommand(()-> pushUpServo.setUp())
                        )
                ),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
                new WaitUntilShootReadyCommand(shooter, waitTime, lowFlywheelTol, highFlywheelTol),
                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed)),
                new WaitCommand(powerFlywheelTime),
                new InstantCommand(() -> continueShoot = false),
                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> pushUpServo.setDown()),
                        new InstantCommand(()-> spindexer.setDefault())
                )
        );
    }
    protected Command shootFromCam(long waitTime){
        return new SequentialCommandGroup(
                new InstantCommand(()-> autoIntakeOn = false),
                new InstantCommand(()-> intake.setDirectPower(0)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DeferredCommand(() -> new InstantCommand(()-> follower.update()), null),
                                new DeferredCommand(() -> new FollowPathCommand(
                                        follower, follower.pathBuilder().addPath(new BezierLine(follower.getPose(), shootPose)).setLinearHeadingInterpolation(follower.getPose().getHeading(), shootPose.getHeading()).build(),
                                        true, 1.0), null)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
//                                setSpindexerCorrect(lineNum),
//                                new WaitCommand(1000),
                                new InstantCommand(() -> continueShoot = true),
                                new InstantCommand(()-> pushUpServo.setUp())
                        )
                ),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
                new WaitUntilShootReadyCommand(shooter, waitTime, lowFlywheelTol, highFlywheelTol),
                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed)),
                new WaitCommand(powerFlywheelTime),
                new InstantCommand(() -> continueShoot = false),
                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> pushUpServo.setDown()),
                        new InstantCommand(()-> spindexer.setDefault())
                )
        );
    }


    protected Command shootFromLines(IntakeLine lineNum, long waitTime){
        return new SequentialCommandGroup(
                new InstantCommand(()-> autoIntakeOn = false),
                new ParallelCommandGroup(
                        getToShootCommand(lineNum),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(()-> intake.setDirectPower(0)),
//                                setSpindexerCorrect(lineNum),
//                                new WaitCommand(1000),
                                new InstantCommand(() -> continueShoot = true),
                                new InstantCommand(()-> pushUpServo.setUp())
                        )
                ),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
                new WaitUntilShootReadyCommand(shooter, waitTime, lowFlywheelTol, highFlywheelTol),
                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed)),
                new WaitCommand(powerFlywheelTime),
                new InstantCommand(() -> continueShoot = false),
                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> pushUpServo.setDown()),
                        new InstantCommand(()-> spindexer.setDefault())
                )
        );
    }
    protected Command shootPreset(long waitTime){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        getToShootCommandPreset(),
                        new SequentialCommandGroup(
//                                setSpindexerCorrect(IntakeLine.MID),
//                                new WaitCommand(1000),
//                                new WaitCommand(500),
                                new InstantCommand(() -> continueShoot = true),
                                new InstantCommand(()-> pushUpServo.setUp())
                        )
                ),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                new WaitUntilShootReadyCommand(shooter, waitTime, lowFlywheelTol, highFlywheelTol),
                new WaitCommand(firstPresetWaitTime),
                new InstantCommand(() -> spindexer.spin(1 * spindexerSpeed)),
                new WaitCommand(powerFlywheelTime),
                new InstantCommand(() -> continueShoot = false),
                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
                new ParallelCommandGroup(
                        new InstantCommand(() -> pushUpServo.setDown()),
                        new InstantCommand(()-> spindexer.setDefault())
                )
        );
    }

    protected InstantCommand setDefaultStartColors(){
//        return new SequentialCommandGroup(
        return new InstantCommand(() -> spindexer.setBallColors(startBallColors));
//        );
    }


    protected Command intake(IntakeLine lineNum){
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(()-> spindexer.setDefault()),
                new InstantCommand(()-> autoIntakeOn = true),
//                new WaitCommand(1000),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                 new InstantCommand(()-> autoIntakeOn = true),
//                new ParallelRaceGroup(

                new ParallelRaceGroup(
//                         autoIntakeCommand(),
                        new AutoIntakeCommand3(spindexer, intake, intakePower, inBetweenTime, useDistanceSensor, hardwareMap),
                        new SequentialCommandGroup(
                                getToLineNum(lineNum),
//                                new WaitCommand(1000),
                                driveToIntakeEnd(lineNum).withTimeout(driveIntakeEndTime),
                                new WaitCommand(1500)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0)),
                new InstantCommand(()-> autoIntakeOn = false),
                new InstantCommand(()-> currSpindexerGotoSpot = 2)
        );
//         }
    }
    protected Command intakeLast(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(()-> spindexer.setDefault()),
                new InstantCommand(()-> autoIntakeOn = true),
//                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
//                new WaitCommand(1000),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                 new InstantCommand(()-> autoIntakeOn = true),
//                new ParallelRaceGroup(
//                         autoIntakeCommand(),
//                         new SequentialCommandGroup(

                new ParallelRaceGroup(
                        new AutoIntakeCommand3(spindexer, intake, cornerIntakePower, inBetweenTime, useDistanceSensor, hardwareMap),
                        new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(() -> currSpindexerGotoSpot = 0),
//                                        new WaitCommand(2000),
//                                        new InstantCommand(() -> currSpindexerGotoSpot = -1)
//                                ),
                                getToLineNum(IntakeLine.CORNER),
//                        ),
                                new FollowPathCommand(follower, toIntakeLineCornerStart, true, intakeCornerDrivePower),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd, true, intakeCornerDrivePower),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, toIntakeLineCornerBack, true, intakeCornerDrivePower),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd2, true, intakeCornerDrivePower),
                                new WaitCommand(500)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0)),
                new InstantCommand(()-> autoIntakeOn = false),
                new InstantCommand(()-> currSpindexerGotoSpot = 2)
        );
    }
    protected Command intakeCorner(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(()-> spindexer.setDefault()),
                new InstantCommand(()-> autoIntakeOn = true),
//                new InstantCommand(()-> spindexer.getTurner().getServo().setPower(0)),
//                new WaitCommand(1000),
                new InstantCommand(() -> currSpindexerGotoSpot = -1),
//                 new InstantCommand(()-> autoIntakeOn = true),
//                new ParallelRaceGroup(
//                         autoIntakeCommand(),
//                         new SequentialCommandGroup(

                new ParallelRaceGroup(
                        new AutoIntakeCommand3(spindexer, intake, cornerIntakePower, inBetweenTime, useDistanceSensor, hardwareMap),
                        new SequentialCommandGroup(
                                getToLineNum(IntakeLine.CORNER),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd, true, intakeCornerDrivePower).withTimeout(cornerTimeout),
                                new WaitCommand(1000),
                                new FollowPathCommand(follower, toIntakeLineCornerBack, true, intakeCornerDrivePower).withTimeout(cornerTimeout),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd2, true, intakeCornerDrivePower).withTimeout(cornerTimeout),
                                new WaitCommand(1000)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0.5)),
                new InstantCommand(()-> autoIntakeOn = false),
                new InstantCommand(()-> currSpindexerGotoSpot = 2)
        );
    }


    protected FollowPathCommand driveToIntakeEnd(IntakeLine lineNum){
        PathChain path = lineNum == IntakeLine.FAR ? toIntakeLineFarEnd : lineNum == IntakeLine.MID ? toIntakeLineMidEnd : lineNum == IntakeLine.CORNER ? toIntakeLineCornerEnd : toIntakeLineCloseEnd;
//        if(lineNum == IntakeLine.FAR) {
//            return new SchedulePathTo(follower, intakeFarStartPose, intakeFarEndPose).setMaxPower(0.3);
//        } else if(lineNum == IntakeLine.MID){
//            return new SchedulePathTo(follower, intakeMidStartPose, intakeMidEndPose).setMaxPower(0.3);
//        } else if(lineNum == IntakeLine.CLOSE) {
//            return new SchedulePathTo(follower, intakeCloseStartPose, intakeCloseEndPose).setMaxPower(0.3);
//        } else{
//            return new SchedulePathTo(follower, intakeCornerStartPose, intakeCornerEndPose).setMaxPower(0.3);
//        }
        return new FollowPathCommand(follower, path, true, intakeDrivePower);
    }



    protected Command park(){
        return new SchedulePathTo(follower, parkPose);
    }

    protected Command getToLineNum(IntakeLine lineNum){
//        Pose endPose;
//        if(lineNum == IntakeLine.FAR){
//            endPose = intakeFarEndPose;
//        }
//        else if(lineNum == IntakeLine.MID) {
//            endPose = intakeMidEndPose;
//        }
//        else if(lineNum == IntakeLine.CLOSE){
//            endPose = intakeCloseEndPose;
//        } else{
//            endPose = intakeCornerEndPose;
//        }


        //  Pose linePose = lineNum == IntakeLine.FAR ? intakeFarStartPose : lineNum == IntakeLine.MID ? intakeMidStartPose : lineNum == IntakeLine.CLOSE ? intakeCloseStartPose : intakeCornerStartPose;
        PathChain path = lineNum == IntakeLine.FAR ? toIntakeLineFarStart : lineNum == IntakeLine.MID ? toIntakeLineMidStart : lineNum == IntakeLine.CORNER ? toIntakeLineCornerStart : toIntakeLineCloseStart;
//        return new ParallelDeadlineGroup(
        return new FollowPathCommand(follower, path, true, 1.0);
//                new AutoIntakeCommand2(spindexer, intake, intakePower, inBetweenTime, useDistanceSensor, maxTimeSwap1, maxTimeSwap2)
//        );
//        return new SchedulePathTo(follower, currentPose, endPose).setMaxPower(0.3);
    }

    protected FollowPathCommand getToShootCommand(IntakeLine intakeLine){
        PathChain path = intakeLine == IntakeLine.FAR ? toShootFromFar : intakeLine == IntakeLine.MID ? toShootFromMid : intakeLine == IntakeLine.CORNER ? toShootFromCorner : toShootFromClose;
        return new FollowPathCommand(follower, path, true, 1.0);
    }
    protected FollowPathCommand getToShootCommandPreset(){
        return new FollowPathCommand(follower, toShootPresets, true, 1.0);
    }


    @Override
    protected void updateTelemetry(){
        // Update pose & follower
        follower.update();
        currentPose = follower.getPose();
//        double currentTime = gameTimer.getTime();

        // Follower
        telemetry.addData("Auto Intake Spot", autoIntakeNum);
        telemetry.addData("Auto Intake On", autoIntakeOn);
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


//
//    public void addStringToTelem(String s, String o){
//        telemetry.addLine(s + o);
//    }
//    public void addToTelemGraph(String s, Number o){
//        telemetryManager.addData(s, o);
//        graphManager.addData(s, o);
//    }
//    public void addToAllTelemGraph(String s, Number o){
//        telemetryManager.addData(s, o);
//        graphManager.addData(s, o);
//        telemetry.addData(s, o);
//        if(dashboard != null) {
//            dashboardPacket.put(s, o);
//        };
//    }
//    public void addBooleanToTelem(String s, boolean o){
//        telemetry.addData(s, o);
//        telemetryManager.addData(s, o);
//    }


}