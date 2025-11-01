package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@TeleOp(group="Subsystems")
public class SpindexerGotoTest extends CommandOpMode {
    public static CRServoEx2.RunMode runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    public static Angle customAngle = Angle.fromDegrees(60);
    public static double finishedTimeThreshold = 1000;

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
        spindexer = new Spindexer(hardwareMap, telemetry, true, false);
        spindexer.initAngle(); // would put this later but oh well
        gp1 = new GamepadEx(gamepad1);

        spot0Button = gp1.getGamepadButton(GamepadKeys.Button.A);
        spot1Button = gp1.getGamepadButton(GamepadKeys.Button.B);
        spot2Button = gp1.getGamepadButton(GamepadKeys.Button.X);

        spot0Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 0),
                new SpindexerGotoSpot(spindexer, 0, runMode, finishedTimeThreshold)
        ));
        spot1Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 1),
                new SpindexerGotoSpot(spindexer, 1, runMode, finishedTimeThreshold)
        ));
        spot2Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 2),
                new SpindexerGotoSpot(spindexer, 2, runMode, finishedTimeThreshold)
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
//        addDataTelemetryGraph("Error", spindexer.getTurner().error);
        addDataTelemetryGraph("Power", spindexer.getTurner().power);
        telemetryM.addData("Test", spindexer.test);
//        telemetryM.addData("Positive power count", spindexer.getTurner().positivePowerCount);
//        telemetryM.addData("Power change count", spindexer.getTurner().powerChangeCount);
//        telemetryM.addData("Set call count", spindexer.getTurner().setCount);
        telemetryM.addData("Revolutions", spindexer.getEncoder().getRevolutions());
        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngle());
        telemetryM.addData("Target Spot", targetSpot);
        telemetryM.addData("Ball Colors", spindexer.getBallColors());
        telemetryM.addData("Loop time (ms)", timer.getDeltaTime());
        telemetryM.update(telemetry);
        graphM.update();
    }
}