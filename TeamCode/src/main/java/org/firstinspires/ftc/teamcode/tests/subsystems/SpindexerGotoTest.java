package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@TeleOp(group="Subsystems")
public class SpindexerGotoTest extends CommandOpMode {
    public static CRServoEx.RunMode runMode = CRServoEx.RunMode.OptimizedPositionalControl;
    public static Angle customAngle = Angle.fromDegrees(60);

    Button spot0Button, spot1Button, spot2Button, customAngleButton;
    Spindexer spindexer;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    int targetSpot = 0;

    @Override
    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap, false);
        spindexer.init(); // would put this later but oh well
        gp1 = new GamepadEx(gamepad1);

        spot0Button = gp1.getGamepadButton(GamepadKeys.Button.A);
        spot1Button = gp1.getGamepadButton(GamepadKeys.Button.B);
        spot2Button = gp1.getGamepadButton(GamepadKeys.Button.X);

        spot0Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 0),
                new SpindexerGotoSpot(spindexer, 0, runMode)
        ));
        spot1Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 1),
                new SpindexerGotoSpot(spindexer, 1, runMode)
        ));
        spot2Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 2),
                new SpindexerGotoSpot(spindexer, 2, runMode)
        ));
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed( // Just to check if gamepad works
                new InstantCommand(() -> targetSpot = -1)
        );

        register(spindexer);
    }

    @Override
    public void run() {
        gp1.readButtons();
        updateTelemetry();
        super.run();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateTelemetry() {
//        telemetryM.addData("Measured Angle", spindexer.getCurrentAngle());
//        telemetryM.addData("Target Spot", targetSpot);
//        telemetryM.addData("Ball Colors", spindexer.getBallColors());
//        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
//        telemetryM.update(telemetry);
//        graphM.update();

        telemetry.addData("Measured Angle", spindexer.getCurrentAngle());
        telemetry.addData("Target Spot", targetSpot);
        telemetry.addData("Ball Colors", spindexer.getBallColors());
        telemetry.addData("Loop time (ms)", timer.getDeltaTime());
        telemetry.update();
    }
}