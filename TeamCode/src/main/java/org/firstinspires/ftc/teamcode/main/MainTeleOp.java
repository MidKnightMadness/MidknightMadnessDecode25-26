package org.firstinspires.ftc.teamcode.main;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.util.Map;

@Config
@Configurable
@TeleOp(name = "Main TeleOp")
public class MainTeleOp extends CommandOpMode {
    Telemetry telemetry;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    Limelight3A limelight3A;
    Follower follower;
    public static Pose startPose = new Pose(8, 8, Math.toRadians(90));
    Timer timer;

    Spindexer spindexer;
    Ramp ramp;
    TwoWheelShooter shooter;

    MotifEnums.Motif pattern = MotifEnums.Motif.NONE;
    Map<String, MotifEnums.Motif> idMap = Map.of(
            "[21]", MotifEnums.Motif.GPP,
            "[22]", MotifEnums.Motif.PGP,
            "[23]", MotifEnums.Motif.PPG
    );
    String motifFileName = "competition/motif_value.txt";
    String botPoseFileName = "competition/robot_pose.txt";
    File motifFile;
    String id;

    double currSpeed = 0.8;
    double maxSpeed = 0.8;
    double midSpeed = 0.5;
    ButtonToggle flywheelToggle;
    ButtonToggle changeSpeed;
    ButtonToggle rampPos;
    boolean flywheelSet = false;
    public static double pShooter = 0.01;
    public static double iShooter = 0;
    public static double dShooter = 0;
    ShootSide side;

    @Override
    public void initialize() {
        timer = new Timer();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();
        limelight3A = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);
        spindexer = new Spindexer(hardwareMap);
        ramp = new Ramp(hardwareMap);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);

        pattern = readMotifFromFile(motifFileName);
        startPose = readPoseFromFile(botPoseFileName);
        pattern = idMap.getOrDefault(id, MotifEnums.Motif.NONE);
        follower = ConstantsBot.createKalmanPinpointAprilFollower(hardwareMap, startPose, telemetry);
        flywheelToggle = new ButtonToggle();
        changeSpeed = new ButtonToggle();
        rampPos = new ButtonToggle();
        shooter.setPid(pShooter, iShooter, dShooter);
    }


    public MotifEnums.Motif readMotifFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(),  fileName);
        if(file.exists()){
            return idMap.get(ReadWriteFile.readFile(file));
        }
        else{
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        return null;
    }

    public Pose readPoseFromFile(String fileName){
        File file = new File(Environment.getExternalStorageDirectory(),  fileName);
        if(file.exists() == false){
            return null;
        }
        String values = ReadWriteFile.readFile(file).trim();
        String[] indivParts = values.split(",");
        try{
           double x = Double.parseDouble(indivParts[0]);
           double y = Double.parseDouble(indivParts[1]);
           double heading = Double.parseDouble(indivParts[2]);
            return new Pose(x, y, heading);
        }
        catch(NumberFormatException n){
            return null;
        }
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y * currSpeed, -gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);

        if(follower.getPose().getX() > 72){
            side = ShootSide.RIGHT;
        }

        if(flywheelToggle.update(gamepad1.right_bumper)){//right bumper -> turn off flywheel
            flywheelSet = !flywheelSet;
        }
        if(flywheelSet){
            schedule(new InstantCommand(() -> shooter.setFlywheelsPower(follower.getPose(), side)));
        }
        else{
            schedule(new InstantCommand(() -> shooter.stopFlywheels()));
        }

        if (changeSpeed.update(gamepad1.b)) {
            currSpeed = currSpeed == maxSpeed ? midSpeed : maxSpeed;
        }





    }


}
