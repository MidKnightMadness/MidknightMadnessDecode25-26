package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerRotateRelative;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "SpindexerSpinAngleTest", group = "Spindexer")
@Disabled
public class SpindexerRotateRelativeTest extends CommandOpMode {
    Spindexer spindexer;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    Timer timer;
    boolean first = true;

    public static double targetAngle = 120;
    @Override
    public void initialize() {
        super.reset(); // Must put first
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        spindexer = new Spindexer(hardwareMap, false).initAngle();

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SpindexerRotateRelative(spindexer, Angle.fromDegrees(targetAngle), 0.5)
        );

        register(spindexer);
    }

    @Override
    public void run() {
        //if (first) schedule(new SpindexerSpinAngle(spindexer, Angle.fromDegrees(360), 1));
        first = false;
        gp1.readButtons();
        updateTelemetry();
        super.run();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
    }

    public void updateTelemetry() {
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngleUnnormalized());
        telemetryM.addData("Runmode", spindexer.getTurner().getRunmode());
        telemetryM.update(telemetry);
    }
}