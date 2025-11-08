package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commands.ShootSequence;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.Map;

@Config
@Configurable
@TeleOp(name = "FlywheelLUTTuning", group = "Flywheel")
public class FlywheelLUTTest extends CommandOpMode {
    Follower follower;
    Pose startPose = new Pose(8, 8, Math.toRadians(90));
    Timer timer;
    MotorEx low;
    MotorEx high;

    public static boolean lowMotorDirForward = true;
    public static boolean highMotorDirForward = false;
    public enum RunMode {
        RawPower,
        VelocityControl
    }
    public static RunMode runMode = RunMode.VelocityControl;
    public static Pose leftShootPose = new Pose(0, 144, Math.toRadians(90));
    public static Pose rightShootPose = new Pose(144, 144, Math.toRadians(90));

    public static double lowVel = 2;
    public static double topVel = 2;

    public final static Map<Double, Double> grToMultiplier = Map.of(
            3., 2.89,
            4., 3.61,
            5., 5.23
    ); // Unused for now

    public static double gearRatio = 4;
    public static ShootSide shootSide = ShootSide.LEFT;

    double currDist = 0;

    double currSpeed = 0.5;
    public static double gamepadVelIncrements = 0.05;

    public static double kP = 0.1;
    public static double kI = 0;
    public static double kD = 0.01;
    public static double kS = 0;
    public static double kV = 0;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    GamepadEx gamepad1Ex;
    Button shootButton;
    ButtonToggle swapShootSides;
    public static long powerFlywheelWaitMs = 1000;
    public static long swapWaitMs = 1000;
    public static long finalWaitMs = 1000;

    Spindexer spindexer;
    Ramp ramp;
    public static MotifEnums.Motif motifOrder = MotifEnums.Motif.PGP;

    @Override
    public void initialize() {
        super.reset();

        low = new MotorEx(hardwareMap, ConfigNames.lowFlywheelMotor);
        high = new MotorEx(hardwareMap, ConfigNames.highFlywheelMotor);
        low.motor.setDirection(lowMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        high.motor.setDirection(highMotorDirForward ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);

//        spindexer = new Spindexer(hardwareMap);
        ramp = new Ramp(hardwareMap);

        follower = ConstantsBot.createKalmanPinpointAprilFollower(hardwareMap, startPose, telemetry);
        timer = new Timer();

        low.setVeloCoefficients(kP, kI, kD);
        high.setVeloCoefficients(kP, kI, kD);
        low.setFeedforwardCoefficients(kS, kV);
        high.setFeedforwardCoefficients(kS, kV);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        graphManager = PanelsGraph.INSTANCE.getManager();

        swapShootSides = new ButtonToggle();
        gamepad1Ex = new GamepadEx(gamepad1);
        shootButton = new GamepadButton(gamepad1Ex, GamepadKeys.Button.RIGHT_BUMPER);

        shootButton.whenPressed(new InstantCommand(() ->{
            SequentialCommandGroup group = (ShootSequence(spindexer, ramp, motifOrder, CRServoEx2.RunMode.OptimizedPositionalControl, 5000));
            group.schedule();
        }));
    }


    @Override
    public void run(){
        super.run();
        gamepad1Ex.readButtons();
        follower.update();
        timer.getTime();
        follower.setTeleOpDrive(-gamepad1.left_stick_y * currSpeed, -gamepad1.left_stick_x * currSpeed, -gamepad1.right_stick_x * currSpeed, true);

        if(swapShootSides.update(gamepad1.b)){
            shootSide = shootSide == ShootSide.LEFT ? ShootSide.RIGHT : ShootSide.LEFT;
        }

        lowVel += gamepad1.right_stick_y * gamepadVelIncrements * timer.getDeltaTime();
        topVel += gamepad1.left_stick_y * gamepadVelIncrements * timer.getDeltaTime();

        // currDist = getDistance(follower.getPose(), shootSide);
        // setFlywheelsPower();
        low.set(lowVel);
        high.set(topVel);
        updateTelem();
    }
    public SequentialCommandGroup ShootSequence(Spindexer spindexer, Ramp ramp, MotifEnums.Motif motif, CRServoEx2.RunMode runMode, double finishedTimeThreshold) {
        ArrayList<Command> commands = new ArrayList<>();
        int[] sequence = spindexer.getOptimalSequence(motif);
        commands.add(new SequentialCommandGroup(
                new InstantCommand(ramp::setRestPos),
                new WaitCommand(powerFlywheelWaitMs)
        ));
        for (int i = 0; i < sequence.length; i++) {
            if (i > 0)
                commands.add(new ParallelCommandGroup(
                        new InstantCommand(ramp::setRestPos),
                        new WaitCommand(swapWaitMs)
                ));
            int spot = sequence[i];
            commands.add(new SequentialCommandGroup(
                    new SpindexerGotoSpot(spindexer, spot, runMode, finishedTimeThreshold),
                    new ParallelCommandGroup(
                            new InstantCommand(ramp::setLowerPos),
                            new InstantCommand(() -> spindexer.removeBall(spot))
                    )
            ));
        }
        commands.add(new SequentialCommandGroup(
                new WaitCommand(finalWaitMs),
                new InstantCommand(ramp:: setRestPos)
        ));
        return new SequentialCommandGroup(commands.toArray(new Command[commands.size()]));
    }


    public double getDistance(Pose robotPose, ShootSide side){
        double xDist = Math.abs(robotPose.getX() - ((side == ShootSide.LEFT) ? leftShootPose.getX() : rightShootPose.getX()));
        double yDist = Math.abs(robotPose.getY() - ((side == ShootSide.LEFT) ? leftShootPose.getY() : rightShootPose.getY()));
        return Math.hypot(xDist, yDist);
    }
    private void updateTelem() {
        addToTelemGraph("Current Distance", currDist);
        addToTelemGraph("Left Velocity", low.getVelocity());
        addToTelemGraph("Right Velocity", high.getVelocity());
        addToTelemGraph("Low Velocity Error", Math.abs(low.getVelocity() - lowVel));
        addToTelemGraph("High Velocity Error", Math.abs(high.getVelocity() - topVel));
        telemetry.update();
        graphManager.update();;
        telemetryManager.update();;
    }

    public void addStringToTelem(String s, String o){
        telemetry.addLine(s + o);
    }
    public void addToTelemGraph(String s, Number o){
        telemetry.addData(s, o);
        telemetryManager.addData(s, o);
        graphManager.addData(s, o);
    }

    public boolean setFlywheelsPower() {//assuming facing the shooting area
        double correctedTopVel = topVel / grToMultiplier.getOrDefault(gearRatio, 3.0);
        double correctedBottomVel = lowVel / grToMultiplier.getOrDefault(gearRatio, 3.0);
        switch (runMode) {
            case VelocityControl:
                low.set(correctedBottomVel); high.set(correctedTopVel);
                break;

            case RawPower:
                low.set(correctedBottomVel / low.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                high.set(correctedTopVel / high.ACHIEVABLE_MAX_TICKS_PER_SECOND);
                break;
        }
        return true;
    }

}

