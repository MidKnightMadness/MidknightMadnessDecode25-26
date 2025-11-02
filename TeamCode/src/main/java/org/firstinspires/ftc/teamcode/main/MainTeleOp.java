package org.firstinspires.ftc.teamcode.main;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.FacePose;
import org.firstinspires.ftc.teamcode.commands.ShootSequence;
import org.firstinspires.ftc.teamcode.commands.SpindexerShootContinuous;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsOldBot;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.tests.subsystems.SpindexerHardcodeTest;
import org.firstinspires.ftc.teamcode.tests.subsystems.SpindexerShootContinuousTest;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.commands.SpindexerSpinAngle;
import org.firstinspires.ftc.teamcode.commands.ShootHardcode;
import java.io.File;
import java.util.Map;
import java.util.StringTokenizer;

@Config
@Configurable
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends CommandOpMode {
//    TelemetryManager telemetryManager;
//    GraphManager graphManager;
    Follower follower;
    public Pose startPose = new Pose(8, 8, Math.toRadians(90));
    Timer timer;

    Spindexer spindexer;
    Ramp ramp;
    TwoWheelShooter shooter;


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
    File motifFile;
    String id;

    double currSpeed = 0.8;
    double maxSpeed = 0.8;
    double midSpeed = 0.5;
    ButtonToggle flywheelFarToggle;
    ButtonToggle flywheelCloseToggle;
    ButtonToggle changeSpeed;
    ButtonToggle spinSpindexter;
    ButtonToggle rampPos;
    boolean flywheelSet = false;
    public static double pShooter = 0.01;
    public static double iShooter = 0;
    public static double dShooter = 0;
//    public static Angle twoBallAngle = new Angle(70, AngleUnit.DEGREES);
    public static Angle lastBallAngle = new Angle(105, AngleUnit.DEGREES);
    ShootSide side;
    public static double[] closeShootPowers = new double[]{
            0.8, 0.6
    };
    public static double[] farShootPowers = new double[]{
            1.0, 1.0
    };

    Pose toCloseLeftShoot = new Pose(72, 72, Math.toRadians(135));
    Pose toCloseRightShoot = new Pose(72, 72, Math.toRadians(135));
    Pose toFarLeftShoot = new Pose(67, 11, Math.toRadians(110));
    Pose toFarRightShoot = new Pose(5, 11, Math.toRadians(70));
    ShootSide shootSide;
    boolean automaticDriving = false;
    public static double headingErrorMax = 0.3;
    public static double pathDistThresholdMax = 3;
    CRServo spindexerServo;
    double currturnerSpeed = 0.3;
    double maxTurnerSpeed = 0.3;
    boolean automaticSpindexer = false;
//    DcMotorSimple.Direction spindexterDir = DcMotorSimple.Direction.FORWARD;

    SpindexerSpinAngle spindexerAngleCommand;
//    Direction spinDexerDir = Direction.FORWARD;
    ShootHardcode spindexerHardcode;
    double dirSpindexer = 1;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        ConstantsBot.motifIsBusy = false;
        timer = new Timer();
//        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphManager = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap);
//        ramp = new Ramp(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.RawPower);

        pattern = MotifEnums.Motif.NONE;
        double robotX = readDoubleFromPose(botXFileName);
        double robotY = readDoubleFromPose(botYFileName);
        double robotHeading = readDoubleFromPose(botHeadingFileName);
//        shootSide = readShootSideFromFile(sideFileName);

        Pose roboPose = new Pose(robotX, robotY, robotHeading);

//        startPose = startPose;

        follower = ConstantsBot.createPinpointFollower(hardwareMap);
        follower.setStartingPose(startPose);
        flywheelFarToggle = new ButtonToggle();
        flywheelCloseToggle = new ButtonToggle();
        changeSpeed = new ButtonToggle();
        spinSpindexter = new ButtonToggle();
        rampPos = new ButtonToggle();
//        shooter.setPid(pShooter, iShooter, dShooter);

        spindexerServo = hardwareMap.get(CRServo.class, ConfigNames.turner);
        follower.startTeleopDrive();
        spindexerAngleCommand = new SpindexerSpinAngle(spindexer, lastBallAngle, 0.2);
        spindexerHardcode = new ShootHardcode(spindexer, shooter, pattern, true);
    }



    public MotifEnums.Motif readMotifFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(),  fileName);
        if(file.exists()){
            return idMap.get(ReadWriteFile.readFile(file));
        }
        else{
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        return MotifEnums.Motif.NONE;
    }
    public double readDoubleFromPose(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if(file.exists()) {
            return Double.parseDouble(ReadWriteFile.readFile(file));
        }
        else {
            return 0;
        }
    }
    public ShootSide readShootSideFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(), fileName);
        if(file.exists()) {
            return fileName.equals("Left") ? ShootSide.LEFT : ShootSide.RIGHT;
        }
        else {
            return ShootSide.LEFT;
        }
    }


    @Override
    public void run(){
        super.run();
        runGamepad1Comands();
        runGamepad2Commands();

        follower.update();
        if(follower.getPose().getHeading() > 0){
            side = ShootSide.RIGHT;
        }


        updateTelem();


    }

    public PathChain getPathChain(Pose pose1, Pose pose2){
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1.getPose(), pose2.getPose()))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setHeadingConstraint(headingErrorMax)
                .setTimeoutConstraint(3000)
                .setTranslationalConstraint(pathDistThresholdMax)
                .build();
        return pathChain;
    }
    private void runGamepad1Comands(){

        if(!automaticDriving){
            follower.setTeleOpDrive(gamepad1.left_stick_y * currSpeed, gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);
        }

        if (changeSpeed.update(gamepad1.right_bumper)) {
            currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
        }

//        if(gamepad1.aWasPressed()){
//            automaticDriving = true;
//            Pose toPose;
//            if(side == ShootSide.LEFT){
//                toPose = toCloseLeftShoot;
//            }
//            else{
//                toPose = toCloseRightShoot;
//            }
//            PathChain pathChain = getPathChain(follower.getPose(), toPose);
//
//            schedule(new FollowPathCommand(follower, pathChain));
//        }

//
//        if(gamepad1.xWasPressed()){
//            automaticDriving = true;
//            Pose toPose;
//            if(side == ShootSide.LEFT){
//                toPose = toFarLeftShoot;
//            }
//            else{
//                toPose = toFarRightShoot;
//            }
//            PathChain pathChain = getPathChain(follower.getPose(), toPose);
//
//            schedule(new FollowPathCommand(follower, pathChain));
//        }

//        if(gamepad1.leftBumperWasPressed()){
//            automaticSpindexer = true;
//            spindexerAngleCommand = new SpindexerSpinAngle(spindexer, twoBallAngle, 0.12);
////            schedule(spindexerAngleCommand);
//        }

        if(automaticSpindexer && spindexerAngleCommand.isFinished()){
            automaticSpindexer = false;
        }

        if(gamepad1.bWasPressed()){
            if(pattern == MotifEnums.Motif.NONE){
                pattern = MotifEnums.Motif.PGP;
            }
            else if(pattern == MotifEnums.Motif.PGP){
                pattern = MotifEnums.Motif.PPG;
            }
            else if(pattern == MotifEnums.Motif.PPG){
                pattern = MotifEnums.Motif.GPP;
            }
            else if(pattern == MotifEnums.Motif.GPP){
                pattern = MotifEnums.Motif.NONE;
            }
        }


    }
    private void runGamepad2Commands(){

        if(flywheelFarToggle.update(gamepad2.right_bumper)){
            shooter.setFlywheelsPower(false);
        }
        else if(flywheelCloseToggle.update(gamepad2.left_bumper)){
            shooter.setFlywheelsPower(true);
        }
        if(gamepad2.right_trigger > 0.5){
            shooter.stopFlywheels();
        }
        if(gamepad2.dpad_up){
            automaticSpindexer = true;
            schedule(spindexerAngleCommand);
        }

//        if(gamepad2.b) {
//            double x;
//            if (follower.getPose().getX() > 72) x = 132;
//            else x = 12;
//            schedule(new FacePose(follower, new Pose(132, 132)));
//        }

        if(!automaticSpindexer){
            spindexerServo.setPower(gamepad2.left_stick_y * currturnerSpeed);
        }

//        if(spinSpindexter.update(gamepad2.left_trigger > 0.5) && !automaticSpindexer) {
//            currturnerSpeed = (currturnerSpeed == 0) ? maxTurnerSpeed : currturnerSpeed;
//            spindexerServo.setPower(dirSpindexer * currturnerSpeed);
//        }
//        if(gamepad2.dpad_down){
//            currturnerSpeed = 0;
//            spindexerServo.setPower(dirSpindexer * currturnerSpeed);
//        }

//        if(gamepad1.left_bumper){
//            automaticSpindexer = true;
//            schedule(spindexerHardcode);
//        }
    }

    private void updateTelem() {
        telemetry.addLine("Automatic Spindexer" + automaticSpindexer);
        addStringToTelem("Motif Pattern", pattern.toString());
        addStringToTelem("Start Pose", startPose.getPose().toString());
        addStringToTelem("Current Pose", follower.getPose().toString());
        addToTelemGraph("Current Speed", currSpeed);
//        addStringToTelem("Shoot Side", side.toString());
        addStringToTelem("Pattern", pattern.toString());
        telemetry.update();
//        graphManager.update();;
//        telemetryManager.update();;
    }
    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
//        telemetryManager.addData(s, o);
//        graphManager.addData(s, o);
    }



}
