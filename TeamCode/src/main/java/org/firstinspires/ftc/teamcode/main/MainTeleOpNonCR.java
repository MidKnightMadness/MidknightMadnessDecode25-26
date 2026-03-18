package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.util.ExtraFns.normAngle;

import android.os.Environment;



import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.graph.GraphManager;
//import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.commands.Robot;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommandNonCR;
import org.firstinspires.ftc.teamcode.commands.spindexer.OutakeSpotsRotation;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPosition;
import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoPositionSmooth;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.SpindexerSpotNonCR;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.newpid.PIDController;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl2;
import org.firstinspires.ftc.teamcode.subsystems.GobildaLightBlock;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PushUpServo;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerNonCR;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.tests.camera.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ExtraFns;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.util.Map;

@Configurable
@Config
@TeleOp(name = "Main TeleOp NON CR", group = "aCompetition")
public class MainTeleOpNonCR extends CommandOpMode {
    Follower follower;
    Pose startPose = new Pose(72, 8, Math.toRadians(90));
    Pose currentPose;
    Timer timer;
    SpindexerNonCR spindexer;
    TwoWheelShooter shooter;
    Intake intake;


    MotifEnums.Motif pattern = MotifEnums.Motif.NONE;
    Map<String, MotifEnums.Motif> idMap = Map.of(
            "21", MotifEnums.Motif.GPP,
            "22", MotifEnums.Motif.PGP,
            "23", MotifEnums.Motif.PPG
    );
    String motifFileName = "competition/motif_value.txt";
    String botXFileName = "competition/robot_x.txt";
    String botYFileName = "competition/robot_y.txt";
    String botHeadingFileName = "competition/robot_heading.txt";
    String sideFileName = "competition/side.txt";
    double maxSpeed = 1.0;
    double midSpeed = 0.5;
    double currSpeed = maxSpeed;
    double intakePower = 1.0;
    double maxIntakePower = 1.0;
//    TelemetryManager telemetryM;
//    GraphManager graphM;

    public static double autoIntakePower = 1;
    public static double  leftOffsetShoot = Math.toRadians(3);
    WheelControl2 wheelControl;
    public static int optimalNumThreads = 8;//for photon
    Pose toCloseLeftShoot = new Pose(57, 94, Math.toRadians(310));
    Pose toCloseRightShoot = new Pose(87, 94, Math.toRadians(230));
    Pose toFarLeftShoot = new Pose(67, 17, Math.toRadians(300));
    Pose toFarRightShoot = new Pose(84, 17, Math.toRadians(240));
    Pose gateIntakeLeft = new Pose(12, 60, Math.toRadians(150));
    Pose gateIntakeRight = new Pose(132, 60, Math.toRadians(30));
    Pose parkRight = new Pose(104, 33, Math.toRadians(90));
    Pose parkLeft = new Pose(144 - 104, 33, Math.toRadians(90));
    ShootSide shootSide = ShootSide.LEFT;
    double rightAprilAngle = 38.565 + 180;//degrees
    double leftAprilAngle = 360 - 38.565;

    boolean autoAlign = false;
    public boolean useArducam = false;
    public static double cameraAlignThresholdDegrees = 5;
    boolean autoSpindexer = false;
    boolean autoDriveToShoot = false;
    boolean autoIntake = false;

    public static double rejectReadingThreshold = 7;
    boolean shootOn;
    boolean readyToShoot = false;
    public static double spindexerCompensationOffset = Math.toRadians(5);//degrees
    int spindexerDirection;

    TwoWheelShooter.RunMode shooterRunMode = TwoWheelShooter.RunMode.VelocityControl;
//    VelocityControl

    PIDController pidAutoAlign = new PIDController(1.0, 0, 0.1);
    //    public static double[] pidAutoAlignAgressive = new double[]{2, 0, 0.1};
    FollowPathCommand followPathCommand;
    double prevHeadingError = 0;
    double turnPower;
    double headingError;
    public static Intake.RunMode intakeRunMode = Intake.RunMode.RawPower;
    public static CRServoEx2.RunMode spindexerRunMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    SpotType activeSpotType = null;
    public static boolean setCustomPower = false;
    public static double customTopTargetVel = 1700;
    public static double customBotTargetVel = 1400;

    public static double offsetRadians = Math.toRadians(0);
    AutoIntakeCommandNonCR autoIntakeCommand;
    SequentialCommandGroup seqAutoIntakeCommand;
    SequentialCommandGroup spindexerGotoPositionSeq;

    SpindexerGotoPositionSmooth spindexerGotoPositionSmooth;

    Telemetry dashboardTelemetry;
    TwoWheelShooter.ShootDist currentShootDist;

    public static boolean useDistanceSensor = true;

    public static double pathDistThresholdMax = 0;
    public static double headingErrorMax = 0;
    public static double timeOutConstraint = 0;
    double targetHeading = 0;


    boolean hasRumbledAllOccupied = false;
    public static boolean readPoseFile = true;//true
    boolean triggerBallShot = false;
    int recentTriggeredSpot = -1;
    BallColor[] currSpindexerBallColors;

    public static double powerAutoIntake = 1.0;

    Pose[] leftGateBounds = new Pose[]{new Pose(14, 52, 0), new Pose(45, 85, 0)};
    Pose[] rightGateBounds = new Pose[]{new Pose(99, 52, 0), new Pose(130, 85, 0)};

    GobildaLightBlock[] spindexerLights;
    GobildaLightBlock pushUpLight;
    GobildaLightBlock readyToShootLight;
    Timer gameTimer;
    AprilTagWebcam arducam;
    double cameraYawRelative = 0;
    double cameraYawGlobal = 0;
    boolean velAgressiveComp = false;
    public static boolean useBulkMode = true;
    PushUpServo pushUpServo;
    public static boolean useDoublePinpoint = false;
    double spindexerRawPower;
    AprilTagDetection tag;
    BallColor spin1Color;
    BallColor spin2Color;
    BallColor spin3Color;
    GobildaLightBlock.Color readyToShootColor;
    GobildaLightBlock.Color pushUpColor;
    public static double autoSettleTime = 0;
    public static long autoIntakeTimeout = 10000;
    public static Pose failsafeLeftPose = new Pose(8.85, 8, Math.toRadians(270));
    public static Pose failsafeRightPose = new Pose(144 - 8.85, 8, Math.toRadians(270));
    double currTurnerPosition;
    double targetSpindexerPosition;
    int activeSpindexerSpot = 0;
    public static double fastSmoothTime = 0.7;
    public static double slowSmoothTime = 1.3;
    boolean driveFieldOriented = false;
    Limelight3A limelight;

    @Override
    public void initialize() {
        //TODO: Bulk read testing

        reset();
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.close();


        timer = new Timer();
        Robot.loadConfig(hardwareMap.appContext, "config.mainBot");

        if (readPoseFile) {
            pattern = readMotifFromFile(motifFileName);
            double robotX = readDoubleFromPose(botXFileName);
            double robotY = readDoubleFromPose(botYFileName);
            double robotHeading = readDoubleFromPose(botHeadingFileName);

            shootSide = readShootSideFromFile(sideFileName);
            startPose = new Pose(robotX, robotY, robotHeading);
        }

        gameTimer = new Timer();

        if (!useDoublePinpoint) {
            follower = ConstantsBot.createPinpointFollower(hardwareMap);
        } else {
            follower = ConstantsBot.createDoublePinpointFollower(hardwareMap);
        }
        follower.setPose(startPose);
        currentPose = startPose;

        initializeSubsystems();

        wheelControl = new WheelControl2(hardwareMap);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();

        register(intake, shooter, spindexer, pushUpServo);

        if(useBulkMode) {
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
            );
//            PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            PhotonCore.experimental.setMaximumParallelCommands(8); // Can be adjusted based on user preference - but raising this number further can cause issues
//            PhotonCore.enable();
        }
        else{
            CommandScheduler.getInstance().setBulkReading(
                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
            );
        }

        spin1Color = BallColor.NONE;
        spin2Color = BallColor.NONE;
        spin3Color = BallColor.NONE;
        telemetry.setMsTransmissionInterval(500);
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboardTelemetry = dashboard.getTelemetry();
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Follower Pose", follower.getPose());
        telemetry.addData("Spindexer Ball Colors", spindexer.getBallColors());
        telemetry.addData("Spindexer Curr Angle", spindexer.getCurrentAngle());
        telemetry.update();
    }

    public void initializeSubsystems() {
        spindexer = new SpindexerNonCR(hardwareMap, useDistanceSensor, new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}, false);
//        spindexer.setMode(spindexerRunMode);
        intake = new Intake(hardwareMap, intakeRunMode);

        shooter = new TwoWheelShooter(hardwareMap, shooterRunMode);

        spindexerLights = new GobildaLightBlock[3];
        spindexerLights[0] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights1));
        spindexerLights[1] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights2));
        spindexerLights[2] = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.spindexerLights3));

        pushUpLight = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.light4));
        readyToShootLight = new GobildaLightBlock(hardwareMap.get(Servo.class, ConfigNames.light5));

        pushUpServo = new PushUpServo(hardwareMap, false);
        if (useArducam) {
            arducam = new AprilTagWebcam();
            arducam.init(hardwareMap, ConfigNames.arducam, telemetry);
        }

//        if (useBulkMode) {
//            CommandScheduler.getInstance().setBulkReading(
//                    hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
//            );

//        } else {
//            CommandScheduler.getInstance().setBulkReading(
//                    hardwareMap, LynxModule.BulkCachingMode.OFF // Scheduler will clean cache for you
//            );
//        }


    }


    public MotifEnums.Motif readMotifFromFile(String fileName) {
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if (file.exists()) {
            return idMap.get(ReadWriteFile.readFile(file));
        } else {
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        return MotifEnums.Motif.NONE;
    }

    public double readDoubleFromPose(String fileName) {
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if (file.exists()) {
            return Double.parseDouble(ReadWriteFile.readFile(file));
        } else {
            return 0;
        }
    }

    public ShootSide readShootSideFromFile(String fileName) {
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if (file.exists()) {
            return ReadWriteFile.readFile(file).equals("Left") ? ShootSide.LEFT : ShootSide.RIGHT;
        } else {
            return ShootSide.LEFT;
        }
    }


    public void updateStartShootDist() {
        if (currentPose != null) {
            currentShootDist = (currentPose.getY() > 20) ? TwoWheelShooter.ShootDist.Close : TwoWheelShooter.ShootDist.Far;
        }
    }

    boolean rumbledLastFive = false;
    boolean start;

    @Override
    public void run() {

        if(!start){
            spindexer.setDirectPosition(0);
            pushUpServo.setDown();
            start = true;
            gameTimer.restart();
        }
        super.run();
        currSpindexerBallColors = spindexer.getBallColors();

        if (!rumbledLastFive && gameTimer.getTime() >= 115000) {//endgame
            gamepad1.rumble(3000);
            gamepad2.rumble(3000);
            rumbledLastFive = true;
        }

        updateLights();
        runGamepad1Comands();
        runGamepad2Commands();
//        if (gamepad1.aWasPressed()) {
//            calibrateYaw();
//        }
        emergencyStops();
        updateTelem();

        if (velAgressiveComp && !shooter.inRecoveryMode) {
            velAgressiveComp = false;
        }


        if (pushUpColor == GobildaLightBlock.Color.GREEN) {
            shootOn = true;
        } else {
            shootOn = false;
        }

        spindexer.updateShootOn(shootOn);




    }


    private void updateLights() {
        if (currSpindexerBallColors != null) {
            if (spin1Color != currSpindexerBallColors[0]) {
                spindexerLights[0].setColor(currSpindexerBallColors[0]);
                spin1Color = currSpindexerBallColors[0];
            }
            if (spin2Color != currSpindexerBallColors[1]) {
                spindexerLights[1].setColor(currSpindexerBallColors[1]);
                spin2Color = currSpindexerBallColors[1];
            }
            if (spin3Color != currSpindexerBallColors[2]) {
                spindexerLights[2].setColor(currSpindexerBallColors[2]);
                spin3Color = currSpindexerBallColors[2];
            }
        }


        if (readyToShootLight != null) {
            GobildaLightBlock.Color targetReadyToShoot = headingError <= Math.toRadians(1) ? GobildaLightBlock.Color.GREEN : GobildaLightBlock.Color.ORANGE;
            if (targetReadyToShoot != readyToShootColor) {
                readyToShootLight.setColor(targetReadyToShoot);
                readyToShootColor = targetReadyToShoot;
            }
        }

        if (pushUpLight != null) {
            GobildaLightBlock.Color targetPushUpColor = useArducam ? GobildaLightBlock.Color.GREEN : GobildaLightBlock.Color.ORANGE;
            if (targetPushUpColor != pushUpColor) {
                pushUpLight.setColor(targetPushUpColor);
                pushUpColor = targetPushUpColor;
            }
        }


    }

    //rumble to notify both gamepads that all balls are occupied
    private void rumbleAllOccuppiedBalls() {
        if (spindexer.allOccuppiedBallColors() && !hasRumbledAllOccupied) {
            gamepad2.rumble(500);
            gamepad1.rumble(500);
            hasRumbledAllOccupied = true;
        } else if (!spindexer.allOccuppiedBallColors()) {
            hasRumbledAllOccupied = false;
        }
    }

    private void emergencyStops() {
        //PWM Disable servo if emergency servo is held
        if(gamepad2.dpad_up){
            if (spindexer.getServoImplEx().isPwmEnabled()) spindexer.getServoImplEx().setPwmDisable();
        } else{
            if(!spindexer.getServoImplEx().isPwmEnabled()) {
                spindexer.getServoImplEx().setPwmEnable();
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            if (autoSpindexer) {
                resetAutoSpindexer();
                if(spindexerGotoPositionSeq != null){
                    CommandScheduler.getInstance().cancel(spindexerGotoPositionSeq);
                }
            }

            if (autoIntake) {
                if (seqAutoIntakeCommand != null) {
                    CommandScheduler.getInstance().cancel(seqAutoIntakeCommand);
                }
                resetAutoIntake();
            }
        }

        if (gamepad1.yWasPressed()) {
            if (autoDriveToShoot) {
                if (followPathCommand != null) {
                    CommandScheduler.getInstance().cancel(followPathCommand);
                }
            }
            follower.breakFollowing();

            followPathCommand = null;
            autoDriveToShoot = false;
        }
    }


    public PathChain getPathChain(Pose pose1, Pose pose2) {
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1.getPose(), pose2.getPose()))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        return pathChain;
    }
    public double calculateAlignTurnPower() {
        double[] aimData = shooter.aimCalculator.targetPowersHeading(
                follower.getPose(),
                follower.getVelocity(),
                TwoWheelShooter.getShootPoseNew(currentPose, shootSide)
        );
        targetHeading = MathFunctions.normalizeAngle(aimData[2] + Math.PI);
        headingError = getAngleError(
                follower.getPose().getHeading(),
                targetHeading
        );

        return -pidAutoAlign.calculate(headingError);
    }

    public double getAngleError(double currentHeading, double targetHeading) {
        //heading is in absolute radians
        double error = targetHeading - currentHeading;
        error = ExtraFns.normAnglePlusMinusPI(error);
        return error;
    }

    private double convertRadToDegrees(double val) {
        return val * 180 / Math.PI;
    }

    boolean swappedToArducam = false;

    double aprilTagBearing = 0;
    boolean detected;
    boolean cameraAlign;

    private void setAlignTurnPower() {
        turnPower = 0;//REMOVE?
        //MODIFY so that the heading is facing the outake side, not the intake side
        if (!autoAlign) return;

        detected = false;
        cameraAlign = false;
        AprilTagDetection detection = null;

        if (useArducam) {
            arducam.update();
            for (AprilTagDetection tag : arducam.getDetectedTags()) {
                detected = (shootSide == ShootSide.LEFT) ? (tag.id == 20) : (tag.id == 24);
                if (detected) detection = tag;
            }
        }

        if (detected && detection != null) {
            aprilTagBearing = Math.toRadians(detection.ftcPose.elevation);
            if(shootSide == ShootSide.LEFT){
                aprilTagBearing += leftOffsetShoot;
            }
            cameraAlign = Math.abs(aprilTagBearing) < Math.toRadians(cameraAlignThresholdDegrees);
        }

        if (cameraAlign) {
            headingError = aprilTagBearing;
        }

        turnPower = calculateAlignTurnPower();
        prevHeadingError = headingError;
    }

    boolean relocalized = false;

    private void calibrateYaw() {
        if (useArducam) {
            arducam.update();
            if (shootSide == ShootSide.LEFT) {
                tag = arducam.getTagBySpecificId(20);
            } else {
                tag = arducam.getTagBySpecificId(24);
            }
            if (tag != null) {
                cameraYawRelative = -tag.ftcPose.pitch;
                cameraYawGlobal = (cameraYawRelative + ((shootSide == ShootSide.LEFT) ? leftAprilAngle : rightAprilAngle));//degrees
                if (smallestAbsDifferenceDegrees(cameraYawGlobal, currentPose.getHeading()) < rejectReadingThreshold) {
                    follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), Math.toRadians(cameraYawGlobal)));
                }
                relocalized = true;
            }
        }
    }

    public double smallestAbsDifferenceDegrees(double a, double b) {
        double diff = Math.abs(a - b) % 360.0;
        if (diff > 180) {
            return 360 - diff;
        }
        return diff;
    }


    public static double getDistance(Pose start, Pose target) {
        double dist = Math.sqrt((target.getY() - start.getY()) * (target.getY() - start.getY()) +
                (target.getX() - start.getX()) * (target.getX() - start.getX()));
        return dist;
    }


    private void runGamepad1Comands() {
        currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));

        if(gamepad1.startWasPressed()){
            useArducam = !useArducam;
        }

        manualResetPose();
        setAlignTurnPower();
        driveRobot();//includes automations
        manualChangeShootSide();
//        manualChangeMotif();
        speedChange();
        toggleAutoAlign();
//        rumbleCloseToGate();

    }

    private void manualResetPose() {
        if (gamepad1.left_trigger > 0.5) {
            follower.setPose(failsafeLeftPose);
        } else if (gamepad1.right_trigger > 0.5) {
            follower.setPose(failsafeRightPose);
        }
    }

    private void rumbleCloseToGate() {
        Pose[] check = null;
        if (shootSide == ShootSide.LEFT) {
            check = leftGateBounds;
        } else {
            check = rightGateBounds;
        }

        if (currentPose.getX() >= check[0].getX() && currentPose.getY() >= check[0].getY()
                && currentPose.getX() <= check[1].getX() && currentPose.getY() <= check[1].getY()) {
            gamepad1.rumbleBlips(2);//not sure if blips is best method here
        }
    }

    private void speedChange() {
        if (gamepad1.rightBumperWasPressed()) {
            currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
        }
    }

    private void toggleAutoAlign() {
        if (gamepad1.leftBumperWasPressed()) {
            autoAlign = !autoAlign;
        }
    }

    private void manualChangeShootSide() {
        if (gamepad1.xWasPressed()) {
            shootSide = shootSide == ShootSide.LEFT ? ShootSide.RIGHT : ShootSide.LEFT;
        }
    }

    //manual changing of motif pattern - in case was not detected
    private void manualChangeMotif() {
        if (gamepad1.bWasPressed()) {
            if (pattern == MotifEnums.Motif.NONE) {
                pattern = MotifEnums.Motif.PGP;
            } else if (pattern == MotifEnums.Motif.PGP) {
                pattern = MotifEnums.Motif.PPG;
            } else if (pattern == MotifEnums.Motif.PPG) {
                pattern = MotifEnums.Motif.GPP;
            } else {
                pattern = MotifEnums.Motif.NONE;
            }
        }
    }

    public void setBallColorsDefault() {
        if (gamepad2.leftStickButtonWasPressed()) {
            spindexer.setDefault();
        }
    }

    private void driveRobot() {

        if (gamepad1.dpadRightWasPressed()) {
            if (followPathCommand != null && !followPathCommand.isFinished()) {
                CommandScheduler.getInstance().cancel(followPathCommand);
            }
            autoDriveToShoot = true;
            Pose toPose = (shootSide == ShootSide.LEFT) ? parkRight : parkLeft;//reversed
            PathChain pathChain = getPathChain(currentPose, toPose);
            followPathCommand = new FollowPathCommand(follower, pathChain).setGlobalMaxPower(1.0);
            schedule(followPathCommand);
        }


        if (!autoDriveToShoot && !driveFieldOriented) {
            wheelControl.drive_relative(-gamepad1.left_stick_y, gamepad1.left_stick_x, !autoAlign ? gamepad1.right_stick_x : turnPower, currSpeed);
        } else if(!autoDriveToShoot && driveFieldOriented){
            wheelControl.driveFieldCentric(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    !autoAlign ? gamepad1.right_stick_x : turnPower,
                    currSpeed,
                    currentPose.getHeading(),
                    shootSide
            );
        }

        follower.update();

        if(gamepad1.rightStickButtonWasPressed()){
            driveFieldOriented = !driveFieldOriented;
        }
    }

    private void runGamepad2Commands() {
        flywheelCommands();
        intakeCommands();
        spindexerCommands();
        pushUpCommands();
        setBallColorsDefault();
//        rumbleAllOccuppiedBalls();
        rumbleReadyToShoot();
        resetSpindexer();
    }

    //MUST Manually reset spindexer at position of 0
    private void resetSpindexer() {
        if (gamepad2.optionsWasPressed()) {
            spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.getTurnerEncoder().encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    boolean aligned = false;

    private void rumbleReadyToShoot() {
        //aligned and right velocity
        readyToShoot = shooter.readyToShoot();
        aligned = autoAlign;
        if (readyToShoot && aligned && headingError <= Math.toRadians(1)) {
            gamepad1.rumble(100);
            gamepad2.rumble(100);
        }
    }

    private void pushUpCommands() {
        if(gamepad1.aWasPressed()){
            pushUpServo.setDown();
        }

        if (gamepad2.bWasPressed()) {
            pushUpServo.setUp();
            if (autoSpindexer) {
                resetAutoSpindexer();
            }
        }
    }

    double autoIntakeSpot = 0;
    boolean autoIntakeFinishedReset;

    private void spindexerCommands() {
        currTurnerPosition = spindexer.getCurrentSpindexerPosition();

        if(autoIntake && autoIntakeCommand != null){
            autoIntakeSpot = autoIntakeCommand.getSpotPosition();
        }

        //MANUAL MOVING BETWEEN SPINDEXER SPOTS
        if (gamepad2.dpadLeftWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            if(activeSpindexerSpot != 0) {
                activeSpindexerSpot--;
            }
            setSpindexerActiveSpot(activeSpindexerSpot);
        }
        else if (gamepad2.dpadRightWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();
            if(activeSpindexerSpot != SpindexerSpotNonCR.getMaxSpots()) {
                activeSpindexerSpot++;
            }
            setSpindexerActiveSpot(activeSpindexerSpot); }
        else if(gamepad2.dpadDownWasPressed() && !autoIntake){
            clearExistingSpindexerCommand();
            if(activeSpindexerSpot != SpindexerSpotNonCR.getMaxSpots() - 1) {
                activeSpindexerSpot += 2;
            }
            setSpindexerActiveSpot(activeSpindexerSpot);
        }
        //RESET POSITION TO 0 BUTTON
        else if (gamepad2.leftBumperWasPressed() && !autoIntake) {
            clearExistingSpindexerCommand();//goes to position of 0 directly
            activeSpindexerSpot = 0;
            setSpindexerActiveSpot(activeSpindexerSpot);
        }

        //SHOOTING BUTTONS
        else if (gamepad2.rightBumperWasPressed() && !autoIntake) {//MEDIUM SPEED
            clearExistingSpindexerCommand(); //goes to position of 0
            activeSpindexerSpot = Math.max(activeSpindexerSpot - 3, 0);
            spindexerGotoPositionSmooth = new SpindexerGotoPositionSmooth(spindexer, SpindexerSpotNonCR.getPositionFromIndex(activeSpindexerSpot, SpotType.INTAKE), fastSmoothTime);
            schedulePosition(spindexerGotoPositionSmooth);
        }  else if(gamepad2.yWasPressed()){//SLOWEST: FOR SORTING
            clearExistingSpindexerCommand();
            activeSpindexerSpot = Math.max(activeSpindexerSpot - 3, 0);
            spindexerGotoPositionSmooth = new SpindexerGotoPositionSmooth(spindexer, SpindexerSpotNonCR.getPositionFromIndex(activeSpindexerSpot, SpotType.INTAKE), slowSmoothTime);
            schedulePosition(spindexerGotoPositionSmooth);
        } else if(gamepad2.shareWasPressed()){//FASTEST
            clearExistingSpindexerCommand();
            activeSpindexerSpot = Math.max(activeSpindexerSpot - 3, 0);
            spindexer.setDirectPosition(SpindexerSpotNonCR.getPositionFromIndex(activeSpindexerSpot, SpotType.INTAKE));
        }

    }

    public void setSpindexerActiveSpot(int activeSpindexerSpot){
        spindexer.setDirectPosition(SpindexerSpotNonCR.getPositionFromIndex(activeSpindexerSpot, SpotType.INTAKE));
    }

    private void clearExistingSpindexerCommand() {
        if (spindexerGotoPositionSeq != null) {
            CommandScheduler.getInstance().cancel(spindexerGotoPositionSeq);
        }
    }

    private void schedulePosition(Command spindexerGotoPosition) {
        autoSpindexer = true;
        spindexerGotoPositionSeq = new SequentialCommandGroup(spindexerGotoPosition,
                new InstantCommand(() -> resetAutoSpindexer()));
        schedule(spindexerGotoPositionSeq);
    }

    public void resetAutoSpindexer() {
        autoSpindexer = false;
        spindexerGotoPositionSmooth = null;
        spindexerGotoPositionSeq = null;
        targetSpindexerPosition = -1;
    }

    private void intakeCommands() {
        if (gamepad2.xWasPressed()) {
            autoIntake = true;
            autoSpindexer = false;

            autoIntakeCommand = new AutoIntakeCommandNonCR(spindexer, intake, powerAutoIntake, autoSettleTime, useDistanceSensor, hardwareMap, SpindexerSpotNonCR.SPOT1, 1);

            seqAutoIntakeCommand = new SequentialCommandGroup(
                    autoIntakeCommand.withTimeout(autoIntakeTimeout),
                    new InstantCommand(() -> resetAutoIntake())
            );

            schedule(seqAutoIntakeCommand);
        }


        if (!autoIntake) {
            intakePower = gamepad2.right_stick_y * maxIntakePower;
            intake.setDirectPower(gamepad2.right_stick_y * maxIntakePower);
        }
    }

    private void resetAutoIntake() {
        autoIntake = false;
        seqAutoIntakeCommand = null;
        autoIntakeCommand = null;
        intake.setDirectPower(0);
    }

    private void flywheelCommands() {
        //shoot sequence command doesn't power the flywheel, need to power using handleShooterInput simaltaenously
        handleShooterInput();
    }

    double currVolt;

    private void handleShooterInput() {
        currVolt = hardwareMap.voltageSensor.iterator().next().getVoltage();


        if (gamepad2.left_trigger > 0.3) {
            pushUpServo.setUp();
            setCurrentShootDist(currVolt);
        } else if (gamepad2.right_trigger > 0.3) {
            shooter.stopFlywheels();
            pushUpServo.setDown();
            setBallColorsDefault();
        }


    }

    private void setCurrentShootDist(double currVolt) {
        if (!setCustomPower) {
            shooter.setFlywheelNew(follower.getPose(), follower.getVelocity(), shootSide, currVolt);
//                shooter.setFlywheelLUT(follower, shootSide, voltageCompensation, currVolt);
        } else {
            shooter.setCustomPower(customBotTargetVel, customTopTargetVel, currVolt);
        }

    }

    //TODO: REORGANIZE TELEMETRY
    private void updateTelem() {
        if (autoIntakeCommand != null && !autoIntakeCommand.isFinished()) {
            telemetry.addData("Curr Target Spot Auto Intake", autoIntakeCommand.currNumSpot);
        }
        telemetry.addData("Relocalized", relocalized);
        if (useDistanceSensor) {
            telemetry.addData("ball detected", spindexer.distCheck);
            telemetry.addData("update rate", 1000.0 / gameTimer.getDeltaTime());


            telemetry.addData("follower velocity X,Y,T", "%f , %f, %f", follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), follower.getVelocity().getTheta());

            telemetry.addData("Shoot Mode", shooterRunMode);
            telemetry.addData("Current Voltage", currVolt);
            telemetry.addData("Start Pose", startPose.getPose().toString());
            telemetry.addData("Start Heading(Deg)", "%.4f", convertRadToDegrees(startPose.getHeading()));
            telemetry.addLine("Current Pose" + currentPose.getX() + "; " + currentPose.getY() + "; " + currentPose.getHeading());

            telemetry.addLine("------------------------------------");


            telemetry.addLine("------------------------------------");
            telemetry.addData("Current Speed", currSpeed);
            telemetry.addData("Shoot Side", shootSide);
            telemetry.addData("Current Shoot Dist", currentShootDist);
            telemetry.addData("Shooter Top Factor", shooter.getCurrTopFactor());
            telemetry.addData("Shooter Bot Factor", shooter.getCurrBotFactor());
            telemetry.addData("Multiplier Top", shooter.getCurrTopFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Multiplier Bot", shooter.getCurrBotFactor() * shooter.getTargetVoltage() / shooter.getCurrVoltage());
            telemetry.addData("Trigger Ball Shot", triggerBallShot);
            telemetry.addData("Recently Triggered Shot", recentTriggeredSpot);
            telemetry.addData("Ready to Shoot", shooter.readyToShoot());
            telemetry.addData("Actual Recovery Time", shooter.getRecoveryTime());


            telemetry.addLine("------------------------------------");
            telemetry.addData("Auto Align", autoAlign);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Heading Error(Alignment)", headingError);
            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Camera Yaw Global", cameraYawGlobal);
            telemetry.addData("Camera Yaw Rel", cameraYawRelative);
            telemetry.addData("Tag", tag == null ? "NONE" : tag.id);
            telemetry.addData("AprilTag Detected", detected);
            telemetry.addData("Bearing", aprilTagBearing);



            telemetry.addLine("------------------------------------");
            telemetry.addData("Auto Spindexer", autoSpindexer);
            telemetry.addData("Auto Drive", autoDriveToShoot);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Intake Run Mode", intakeRunMode);
            telemetry.addData("Intake Power", intakePower);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Shooter Mode", shooterRunMode);
            telemetry.addData("Shooter Top Power", shooter.high.get());
            telemetry.addData("Shooter Bot Power", shooter.low.get());
            telemetry.addData("Shooter Top Vel", shooter.high.getVelocity());
            telemetry.addData("Shooter Bot Vel", shooter.low.getVelocity());
            telemetry.addData("Corr Shooter Top", shooter.high.getCorrectedVelocity());
            telemetry.addData("Corr Shooter Bot", shooter.low.getCorrectedVelocity());

            telemetry.addData("Distance From Goal", shooter.getDistance(currentPose, shootSide));


            telemetry.addLine("--------------------------------");
            telemetry.addData("Spindexer Mode", spindexerRunMode);
            telemetry.addData("Spindexer Angle", spindexer.getCurrentAngle());
            telemetry.addData("Curr Active Spot", activeSpindexerSpot);
            telemetry.addData("Auto Intake Spot", autoIntakeSpot);
            if (currSpindexerBallColors != null) {
                telemetry.addData("Spindexer Ball Color 0", currSpindexerBallColors[0]);
                telemetry.addData("Spindexer Ball Color 1", currSpindexerBallColors[1]);
                telemetry.addData("Spindexer Ball Color 2", currSpindexerBallColors[2]);
            }
            telemetry.addData("Color 1", spin1Color);
            telemetry.addData("Color 2", spin2Color);
            telemetry.addData("Color 3", spin3Color);

            telemetry.addData("Spindexer Direction", spindexerDirection);

            telemetry.addData("Spindexer Raw Power", spindexerRawPower);
            telemetry.addData("Spindexer Curr Spot Type", activeSpotType);
            telemetry.addData("Dist 1", spindexer.getDistance1());
            telemetry.addData("Dist 2", spindexer.getDistance2());

            telemetry.addData("Shooter Recovery Gains", shooter.inRecoveryMode);
            telemetry.addData("Error Bot", shooter.bottomError);
            telemetry.addData("Error Top", shooter.topError);
            telemetry.update();
//            dashboardTelemetry.addData("Shooter Top Vel", shooter.high.getVelocity());
//            dashboardTelemetry.addData("Shooter Bot Vel", shooter.low.getVelocity());
//            dashboardTelemetry.addData("Corr Shooter Top", shooter.high.getCorrectedVelocity());
//            dashboardTelemetry.addData("Corr Shooter Bot", shooter.low.getCorrectedVelocity());
//            dashboardTelemetry.update();

        }

    }
}