package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.commands.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(group="Subsystems")
public class SpindexerGotoTest extends OpMode {
    public static CRServoEx.RunMode runMode = CRServoEx.RunMode.OptimizedPositionalControl;
    public static Angle customAngle = Angle.fromDegrees(60);

    Button spot0Button, spot1Button, spot2Button, customAngleButton;
    Spindexer spindexer;
    GamepadEx driverOp;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    int targetSpot;

    @Override
    public void init() {
        PanelsDrawing.init();
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap);
        driverOp = new GamepadEx(gamepad1);

        spot0Button = driverOp.getGamepadButton(GamepadKeys.Button.A);
        spot1Button = driverOp.getGamepadButton(GamepadKeys.Button.B);
        spot2Button = driverOp.getGamepadButton(GamepadKeys.Button.X);
        customAngleButton = driverOp.getGamepadButton(GamepadKeys.Button.Y);

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
    }

    @Override
    public void start() {
        timer.restart();
        spindexer.init();
    }

    @Override
    public void loop() {
        updateTelemetry();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
        graphM.addData(key, value);
    }

    public void updateTelemetry() {
        telemetryM.addData("Measured Angle", spindexer.getCurrentAngle());
        telemetryM.addData("Target Spot", targetSpot);
        telemetryM.addData("Ball Colors", spindexer.getBallColors());
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
        graphM.update();
    }
}