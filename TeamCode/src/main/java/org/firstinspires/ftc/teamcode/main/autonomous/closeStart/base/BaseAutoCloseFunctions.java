package org.firstinspires.ftc.teamcode.main.autonomous.closeStart.base;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandMotif;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandNonCR;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootUpdateCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.WaitUntilShootReadyCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPosition;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPositionSmooth;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.commands.pathing.BuildPath;
import org.firstinspires.ftc.teamcode.game.IntakeLine;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.main.autonomous.BaseAuto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.commands.CamCommand;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;

@Config
@Configurable
//Far side base for other autos. includes all necessary functions & poses & commands
public class BaseAutoCloseFunctions extends BaseAuto {
    int objectDetectionPipeline = 3;
    Pose startPose = new Pose(118, 129, Math.toRadians(225));
    Pose shootPose = new Pose(88, 89, Math.toRadians(230));
    Pose inwardShootPose = new Pose(86, 105, Math.toRadians(215));
    Pose parkPose = new Pose(129, 129, Math.toRadians(0));
    Pose openGatePose = new Pose(128, 76, Math.toRadians(180));
    Pose gateIntakePose = new Pose(131, 61, Math.toRadians(20));
    Pose intakeCloseStartPose = new Pose(95, 84, Math.toRadians(0));
    Pose intakeCloseEndPose = new Pose(123, 84, Math.toRadians(0));
    Pose intakeMidStartPose = new Pose(97, 56, Math.toRadians(0));
    Pose intakeMidEndPose = new Pose(130, 56, Math.toRadians(0));

    Pose intakeFarStartPose = new Pose(97, 34, Math.toRadians(0));
    Pose intakeFarEndPose = new Pose(130, 34, Math.toRadians(0));
    Pose intakeCornerStartPose = new Pose(121, 10, Math.toRadians(0));
    Pose intakeCornerEndPose = new Pose(127, 10, Math.toRadians(0));
    Pose intakeCornerStartPose2 = new Pose(123, 6, Math.toRadians(0));
    Pose intakeCornerEndPose2 = new Pose(129, 6, Math.toRadians(0));
    Pose closeShootPose = new Pose(91.2, 84.6, Math.toRadians(230));

    Pose startDetectPose = new Pose(110, 10, Math.toRadians(0));
    Pose strafeFourPose = new Pose(110, 40, Math.toRadians(0));
    Pose lastPickupStart = new Pose(97, 129, Math.toRadians(0));
    Pose lastPickupEnd = new Pose(130, 129, Math.toRadians(0));


    public static long driveIntakeEndTime = 4000;


    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    MotifEnums.Motif currentIntakeOrder = MotifEnums.Motif.NONE;
    ShootSide shootSide = ShootSide.RIGHT;
    Pose currentPose = startPose;

    public static double pathDistThresholdMin = 0;
    public static double headingError = 0;
    public static double timeOutConstraint = 0;
    public static double velConstraint = 0;

    //TODO: TRY VELOCITY CONSTRAINT
    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;

    private final BallColor[] startBallColors = new BallColor[] {BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
    SpindexerSpot[] spots;
    public static boolean useBulkMode = true;
    PathChain toShootPresets;
    PathChain toIntakeLineFarStart;
    PathChain toIntakeLineFarEnd, toIntakeLineMidStart, toIntakeLineMidEnd, toIntakeLineCornerEnd;
    PathChain toIntakeLineCornerStart, toShootFromFar, toShootFromMid, toShootFromClose;
    PathChain backupMid;
    PathChain toShootFromCorner, toIntakeLineCloseStart,  toIntakeLineCloseEnd;
    PathChain toIntakeLineCornerBack,  toIntakeLineCornerEnd2,  toShootCloseFromMid;
    PathChain toIntakeLastPath, toShootFromLast,  toShootCloseFromClose;

    PathChain toShootCloseFromFar, lastPathIntake, openGatePostMid, openGatePostClose;
    PathChain toGateIntake, gateIntakeToShoot, closeLineToCloseIn, midLineToCloseIn, farLineToCloseIn;
    PathChain toPark;
    public static long maxShootingTime = 3000;

    public static double strafePower = 0.5;
    AprilTagDetection tag21;
    AprilTagDetection tag22;
    AprilTagDetection tag23;
    int motifTag = 23;
    PushUpServo pushUpServo;
    boolean isReadyToShoot;

    public void chooseSideConstraints(){
        if(getShootSide() == ShootSide.LEFT) {
            Pose rightShootPose = shootPose;
            shootPose = new Pose(144- rightShootPose.getX(), rightShootPose.getY(), normAngle(Math.PI - Math.toRadians(245)));
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
            lastPickupStart = applyLeft(lastPickupStart);
            lastPickupEnd = applyLeft(lastPickupEnd);
            shootSide = ShootSide.LEFT;
            targetx1 = 144 - targetx1;
            targetx2 = 144 - targetx2;
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
        if(getShootSide() == ShootSide.LEFT){
            return applyLeft(startPose);
        } return startPose;
    }

    @Override
    public void setupVision(){
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, ConfigNames.arducam);

        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(objectDetectionPipeline);
        limelight.start();

    }

    @Override
    protected ShootSide getSide(){
        return shootSide;
    }

    boolean useDistanceSensor = true;
    public static double inBetweenTime = 0;
    int aprilTagID = 0;
    BuildPath buildPath;
    public static long timeoutCorner = 100;
    public static double intakeDrivePower = 0.5;
    public static double intakeCornerDrivePower = 0.5;
    public static double intakePower = 1;
    public static double autoCameraDrivePower = 0.4;//primary
    public static double autoCameraDrivePowerSec = 0.8;
    public static double maxTimeSwap1 = 1000;
    public static double maxTimeSwap2 = 1000;
    public static long maxWaitTillShoot = 3000;
    public static double targetx1 = 130;
    public static double targetx2 = 134;
    public static double cornerIntakePower = 1.0;
    public static double lowFlywheelTol = 100;
    public static double highFlywheelTol = 100;
    public static double totalSmoothTime = 1;//what we have for teleop(fast) -> 0.7, (slow) - 1.0
    PathChain driveToStartPath;
    PathChain strafe1Path;
    boolean camDetect = false;
    public static long firstPresetWaitTime = 2500;
    public static long cornerTimeout = 1800;
    public static long betweenShootPresetTime = 200;
    @Override
    public void initialize_loop(){
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
        telemetry.addData("Spindexer angle", spindexer.getTurner().getAngle());
        telemetry.addData("Spindexer spot", spindexer.getNearestIntakePosition(SpotType.INTAKE));
        telemetry.update();
    }

    Map<Integer, MotifEnums.Motif> idMap = Map.of(
            21, MotifEnums.Motif.GPP,
            22, MotifEnums.Motif.PGP,
            23, MotifEnums.Motif.PPG
    );
    //keep these empty and build the path using follower's current Pose


    @Override
    protected void buildPaths(){
        chooseSideConstraints();

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

        openGatePostMid = buildPath(intakeMidEndPose, openGatePose);
        openGatePostClose = buildPath(intakeCloseEndPose, openGatePose);
        toShootFromMid = buildPath(openGatePose, shootPose);

        toIntakeLineCloseStart = buildPath(shootPose, intakeCloseStartPose);
        toIntakeLineCloseEnd = buildPath(intakeCloseStartPose, intakeCloseEndPose);

        closeLineToCloseIn = buildPath(intakeCloseEndPose, inwardShootPose);
        midLineToCloseIn = buildPath(intakeMidStartPose, inwardShootPose);
        backupMid = buildPath(intakeMidEndPose, intakeMidStartPose);
        farLineToCloseIn = buildPath(intakeFarEndPose, inwardShootPose);



        toShootCloseFromFar = buildPath(intakeFarEndPose, closeShootPose);
        toShootCloseFromMid = buildPath(intakeMidEndPose, closeShootPose);
        toShootCloseFromClose = buildPath(intakeFarEndPose, closeShootPose);

        toPark = buildPath(shootPose, parkPose);

        toGateIntake = buildPath(shootPose, gateIntakePose);
        gateIntakeToShoot = buildPath(gateIntakePose, shootPose);

        driveToStartPath = buildPath(shootPose, startDetectPose);
        toIntakeLastPath = buildPath(shootPose, lastPickupStart);
        lastPathIntake = buildPath(lastPickupStart, lastPickupEnd);
        toShootFromLast = buildPath(lastPickupEnd, shootPose);
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
    AutoIntakeCommandNonCR autoIntakeCommand;
    int autoIntakeSpot = -1;
    int autoIntakeIndex = -1;
    CamCommand cam;
    CamCommand cam2;
    boolean continueShoot;
    boolean prevContinueShoot;
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

//        if(autoIntakeCommand != null){
//            autoIntakeSpot = autoIntakeCommand.getCurrSpot();
//            autoIntakeIndex = autoIntakeCommand.getCurrentIndex();
//        }


        if(continueShoot){
            double currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();
            shooter.setFlywheelStaticLUT(follower.getPose(), shootSide, true, currVolt);
        } else if(!continueShoot && prevContinueShoot){
            shooter.stopFlywheels();
        }
        prevContinueShoot = continueShoot;

    }

    @Override
    protected void initializeMechanisms() {
        spindexer = new SpindexerNonCR(hardwareMap, useDistanceSensor, startBallColors, true);
        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);
        pushUpServo = new PushUpServo(hardwareMap);
        register(intake, shooter, spindexer, pushUpServo);
        telemetry.setMsTransmissionInterval(500);
    }





    @Override
    protected void setBulkReading(){
        //wait 2 seconds for the spindexer to get to position 0
        waitThread(2000);

        //reset spindexer encoders then wait 2 seconds
        spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitThread(2000);
        spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(1).getIntakePositionSolo());
        waitThread(500);
        spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(2).getIntakePositionSolo());
        waitThread(500);
        spindexer.setDirectPosition(SpindexerSpotNonCR.fromIndex(3).getIntakePositionSolo());
        waitThread(500);

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

    public void waitThread(long time){
        try{
            Thread.sleep(time);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }



    @Override
    protected Command postMotifSequence(){
        return new InstantCommand();
    }



    protected FollowPathCommand getToShootCloseCommand(IntakeLine intakeLine, boolean inner){
        PathChain path;
        if(inner) {
            path = intakeLine == IntakeLine.FAR ? toShootCloseFromFar : intakeLine == IntakeLine.MID ? toShootCloseFromMid : toShootCloseFromClose;
        } else{
            path = intakeLine == IntakeLine.FAR ? closeLineToCloseIn  : intakeLine == IntakeLine.MID ? midLineToCloseIn : farLineToCloseIn;
        }
        return new FollowPathCommand(follower, path, true, 1.0);
    }

    protected Command backupMid(){
        return new FollowPathCommand(follower, backupMid, true, 1.0);
    }

    protected Command shootClose(IntakeLine lineNum, double maxWaitTime, boolean inner){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        getToShootCloseCommand(lineNum, inner),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                new InstantCommand(()-> pushUpServo.setUp()),
                                new InstantCommand(()-> continueShoot = true)
                        )
                ),
                new InstantCommand(()-> continueShoot = true),
                new WaitUntilShootReadyCommand(shooter, maxWaitTime, lowFlywheelTol, highFlywheelTol),
                new SpindexerGotoPositionSmooth(spindexer, spindexer.endOutakePosition, totalSmoothTime)
        ).andThen(new InstantCommand(()-> continueShoot = false));
    }

    protected Command shootFromGate(){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, gateIntakeToShoot, 1),
                        new SequentialCommandGroup(
                                new InstantCommand(()-> pushUpServo.setUp()),
                                new InstantCommand(()-> continueShoot = true)
                        )
                ),
                new InstantCommand(()-> continueShoot = true),
                new WaitUntilShootReadyCommand(shooter, firstPresetWaitTime, lowFlywheelTol, highFlywheelTol),
                new SpindexerGotoPositionSmooth(spindexer, spindexer.endOutakePosition, totalSmoothTime)
        ).andThen(new InstantCommand(()-> continueShoot = false));
    }




    protected Command openGateMid(){
        return new FollowPathCommand(follower, openGatePostMid, 0.7);
    }

    protected Command openGateClose(){
        return new FollowPathCommand(follower, openGatePostClose, 0.7);
    }
    protected Command intakeFromGate(){
        autoIntakeCommand = new AutoIntakeCommandNonCR(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap, SpindexerSpotNonCR.SPOT1, 1);
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(()-> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new ParallelRaceGroup(
                        autoIntakeCommand,
                        new SequentialCommandGroup(
                            new FollowPathCommand(follower, toGateIntake, 1),
                            new WaitCommand(1000)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0)
                ));
    }

    protected Command shootPresetUnsorted(){
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        getToShootCommandPreset(),
                        new SequentialCommandGroup(
                                new InstantCommand(()-> pushUpServo.setUp()),
                                new InstantCommand(()-> continueShoot = true)
                        )
                ),
                new InstantCommand(()-> continueShoot = true),
                new WaitUntilShootReadyCommand(shooter, firstPresetWaitTime, lowFlywheelTol, highFlywheelTol),
                new SpindexerGotoPositionSmooth(spindexer, spindexer.endOutakePosition, totalSmoothTime)
        ).andThen(new InstantCommand(()-> continueShoot = false));
    }
    int presetSortIndexStart = 0;
    protected Command shootPresetSorted(){
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new InstantCommand(()-> presetSortIndexStart = 2),
                        new InstantCommand(()-> presetSortIndexStart = 3),
                        ()-> motifPattern == MotifEnums.Motif.GPP
                ),

                new ParallelDeadlineGroup(
                        getToShootCommandPreset(),
                        new SequentialCommandGroup(
                                new DeferredCommand(() ->new SpindexerGotoPosition(spindexer, SpindexerSpotNonCR.fromIndex(presetSortIndexStart).getIntakePositionSolo()), null),
                                new InstantCommand(()-> continueShoot = true)
                        )
                ),
                new InstantCommand(()-> continueShoot = true),
                new InstantCommand(()-> pushUpServo.setUp()),
                new WaitUntilShootReadyCommand(shooter, maxWaitTillShoot, lowFlywheelTol, highFlywheelTol),
                new ConditionalCommand(
                        new SequentialCommandGroup(//GPP
                                new SpindexerGotoPosition(spindexer, SpindexerSpotNonCR.fromIndex(0).getOuttakePositionSolo()),
                                new WaitCommand(betweenShootPresetTime),
                                new SpindexerGotoPosition(spindexer,  SpindexerSpotNonCR.fromIndex(2).getOuttakePositionSolo()),
                                new WaitCommand(betweenShootPresetTime),
                                new SpindexerGotoPosition(spindexer,  SpindexerSpotNonCR.fromIndex(1).getOuttakePositionSolo())
                        ),
                        new ConditionalCommand(
                                new SequentialCommandGroup(//PPG
                                        new SpindexerGotoPosition(spindexer, SpindexerSpotNonCR.fromIndex(1).getOuttakePositionSolo()),
                                        new WaitCommand(betweenShootPresetTime),
                                        new SpindexerGotoPosition(spindexer,  SpindexerSpotNonCR.fromIndex(2).getOuttakePositionSolo()),
                                        new WaitCommand(betweenShootPresetTime),
                                        new SpindexerGotoPosition(spindexer,  SpindexerSpotNonCR.fromIndex(3).getOuttakePositionSolo())
                                ),
                                new SpindexerGotoPositionSmooth(spindexer, spindexer.endOutakePosition, totalSmoothTime),//PGP
                                () -> motifPattern == MotifEnums.Motif.PPG
                        ),
                        () -> motifPattern == MotifEnums.Motif.GPP
                )
        ).andThen(new InstantCommand(()-> continueShoot = false));
    }



    protected InstantCommand setDefaultStartColors(){
        return new InstantCommand(() -> spindexer.setBallColors(startBallColors));
    }



    protected Command intake(IntakeLine lineNum, boolean sort){
        autoIntakeCommand = new AutoIntakeCommandNonCR(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap, SpindexerSpotNonCR.SPOT1, 1);
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(() -> currentIntakeOrder = getIntakeOrder(lineNum)),
                new InstantCommand(()-> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new ParallelRaceGroup(
                        new ConditionalCommand(
                                new AutoIntakeCommandMotif(spindexer, intake, intakePower, inBetweenTime, motifPattern, currentIntakeOrder, hardwareMap),
                                autoIntakeCommand,
                                ()-> sort
                        ),
                        new SequentialCommandGroup(
                            getToLineNum(lineNum),
                            driveToIntakeEnd(lineNum).withTimeout(driveIntakeEndTime),
                            new WaitCommand(1000),
                            getToLineNum(lineNum),
                            driveToIntakeEnd(lineNum).withTimeout(driveIntakeEndTime)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0)
                ));
    }


    protected Command intakeCorner(boolean sort){
        return new SequentialCommandGroup(
                new InstantCommand(() -> pushUpServo.setDown()),
                new InstantCommand(() -> currentIntakeOrder = MotifEnums.Motif.PGP),
                new InstantCommand(()-> spindexer.setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE})),
                new ParallelRaceGroup(
                        new ConditionalCommand(
                                new AutoIntakeCommandMotif(spindexer, intake, intakePower, inBetweenTime, motifPattern, currentIntakeOrder, hardwareMap),
                                new AutoIntakeCommandNonCR(spindexer, intake, intakePower, inBetweenTime, true, hardwareMap, SpindexerSpotNonCR.SPOT1, 1),
                                () -> sort
                        ),
                        new SequentialCommandGroup(
                                getToLineNum(IntakeLine.CORNER),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd, true, 0.6).withTimeout(2500),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, toIntakeLineCornerBack, true, 1).withTimeout(1500),
                                new FollowPathCommand(follower, toIntakeLineCornerEnd2, true, intakeCornerDrivePower).withTimeout(1000),
                                new WaitCommand(800)
                        )
                ),
                new InstantCommand(()-> intake.setDirectPower(0))
        );
    }

    protected MotifEnums.Motif getIntakeOrder(IntakeLine lineNum){
        if(lineNum == IntakeLine.CORNER || lineNum == IntakeLine.MID){
            return MotifEnums.Motif.PGP;
        } else if(lineNum == IntakeLine.FAR){
            return MotifEnums.Motif.GPP;
        } else{
            return MotifEnums.Motif.PPG;
        }
    }

    protected FollowPathCommand driveToIntakeEnd(IntakeLine lineNum){
        PathChain path = lineNum == IntakeLine.FAR ? toIntakeLineFarEnd : lineNum == IntakeLine.MID ? toIntakeLineMidEnd : lineNum == IntakeLine.CORNER ? toIntakeLineCornerEnd : toIntakeLineCloseEnd;
        return new FollowPathCommand(follower, path, true, intakeDrivePower);
    }



    protected Command park(){
        return new SchedulePathTo(follower, parkPose);
    }

    protected Command getToLineNum(IntakeLine lineNum){
        PathChain path = lineNum == IntakeLine.FAR ? toIntakeLineFarStart : lineNum == IntakeLine.MID ? toIntakeLineMidStart : lineNum == IntakeLine.CORNER ? toIntakeLineCornerStart : toIntakeLineCloseStart;
        return new FollowPathCommand(follower, path, true, 1.0);
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
        currentPose = follower.getPose();

//        telemetry.addData("Auto Intake Spot(0-3)", autoIntakeSpot);
//        telemetry.addData("Auto Intake Seq Index(0-2)", autoIntakeIndex);
//        telemetry.addData("Current Intake Order", currentIntakeOrder);
        telemetry.addData("Update Rate", 1000.0 / gameTimer.getDeltaTime());
        telemetry.addData("Current Follower Pose", currentPose.getPose());
        telemetry.addData("Follower Velocity", follower.getVelocity());

        telemetry.addData("Encoder position", spindexer.getEncoder().getPosition());
        telemetry.addData("Encoder angle", spindexer.getEncoder().getAngle());

        telemetry.addData("Cam detect", camDetect);

        if(autoIntakeCommand != null){
            telemetry.addData("Curr Spot", autoIntakeCommand.currNumSpot);
            telemetry.addData("At Spot", autoIntakeCommand.atSpot);

        }
        telemetry.addData("Motif", motifPattern);
        if(spindexer.getBallColors() != null) {
            telemetry.addData("Spindexer Ball Color 0", spindexer.getBallColors()[0]);
            telemetry.addData("Spindexer Ball Color 1", spindexer.getBallColors()[1]);
            telemetry.addData("Spindexer Ball Color 2", spindexer.getBallColors()[2]);
        }
        telemetry.addData("Dist 1", spindexer.getDistance1());
        telemetry.addData("Dist 1", spindexer.getDistance2());
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
            telemetry.addData("Spindexer Ball Colors Spot", spindexer.getBallColors());
        }

        telemetry.update();

    }


}
