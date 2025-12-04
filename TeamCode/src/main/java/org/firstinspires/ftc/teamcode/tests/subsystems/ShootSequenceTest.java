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
    public static int[] shootArray = new int[]{0, 1, 2};
    Spindexer spindexer;
    TwoWheelShooter shooter;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    Follower follower;
    SpindexerSpot[] spots;
    public static Pose startPose = new Pose(0, 0, Math.toRadians(90));
    public static ShootSide shootSide = ShootSide.RIGHT;
    @Override
    public void initialize() {
        super.reset();
        follower = ConstantsBot.createPinpointFollowerCustom(hardwareMap, startPose);
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );
        SpindexerSpot[] spots = SpindexerSpot.convertFromindex(shootArray);

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap, false);
        spindexer.initAngle(); // would put this later but oh well
        gp1 = new GamepadEx(gamepad1);

        shootSeqButton = gp1.getGamepadButton(GamepadKeys.Button.A);

        shootSeqButton.whenPressed(new SequentialCommandGroup(
                new ShootSeqCommand(spindexer, shooter, spots, follower, shootSide, true)
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
        addDataTelemetryGraph("Top Target Vel", shooter.getPredictedTopVel());
        addDataTelemetryGraph("Top Velocity", shooter.high.getVelocity());

        addDataTelemetryGraph("Bot Target Vel", shooter.getPredictedBotVel());
        addDataTelemetryGraph("Bot Velocity", shooter.low.getVelocity());

        telemetry.addData("Dist From Goal", shooter.getDistance(follower.getPose(), shootSide));
        telemetryM.addData("Revolutions", spindexer.getEncoder().getRevolutions());
        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngle());
        telemetry.addData("Spindexer Current Angle", spindexer.getCurrentAngle());
        telemetryM.addData("Ball Colors", spindexer.getBallColors());
        telemetryM.addData("Loop time (ms)", timer.getDeltaTime());
        telemetryM.update(telemetry);
        graphM.update();
    }
}