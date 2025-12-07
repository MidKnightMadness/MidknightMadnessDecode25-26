package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootSeqCommand;
import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.game.ShootSide;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@TeleOp(group="Subsystems")
public class ShootSequenceTest extends CommandOpMode {
    public static CRServoEx2.RunMode runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    public static Angle currAngle;

    Button shootSeqButton;
//    public static int[] shootArray = new int[]{0, 1, 2};
    Spindexer spindexer;
    TwoWheelShooter shooter;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    Follower follower;
    public static Pose startPose = new Pose(72, 3, Math.toRadians(90));
    public static ShootSide shootSide = ShootSide.RIGHT;
    public static BallColor[] ballColors = new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
    public static MotifEnums.Motif motif = MotifEnums.Motif.GPP;
    @Override
    public void initialize() {
        super.reset();
        follower = ConstantsBot.createPinpointFollowerCustom(hardwareMap, new Pose(0, 0, 0));
        follower.setPose(startPose);
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );
        spindexer.setBallColors(ballColors);
        SpindexerSpot[] spots = spindexer.getOptimalSequence(motif);

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap, false);
        spindexer.initAngle(); // would put this later but oh well
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.VelocityControl);

        gp1 = new GamepadEx(gamepad1);

        shootSeqButton = gp1.getGamepadButton(GamepadKeys.Button.A);

        shootSeqButton.whenPressed(new SequentialCommandGroup(
                new ShootSeqCommand(spindexer, shooter, spots, follower, shootSide, false, TwoWheelShooter.ShootDist.Close)
        ));

        register(spindexer, shooter);
        follower.startTeleopDrive();

    }

    @Override
    public void run() {
        follower.update();
        gp1.readButtons();
        updateTelemetry();
        super.run();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateTelemetry() {
        telemetry.addData("Top Target Vel", shooter.getPredictedTopVel());
        telemetry.addData("Top Velocity", shooter.high.getVelocity());

        telemetry.addData("Bot Target Vel", shooter.getPredictedBotVel());
        telemetry.addData("Bot Velocity", shooter.low.getVelocity());

        telemetry.addData("Dist From Goal", shooter.getDistance(follower.getPose(), shootSide));
        telemetry.addData("Revolutions", spindexer.getEncoder().getRevolutions());
//        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngle());
        telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle());
        telemetry.addData("Ball Colors", spindexer.getBallColors());
        telemetry.addData("Loop time (ms)", timer.getDeltaTime());
        telemetry.update();
//        graphM.update();
    }
}