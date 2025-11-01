package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ShootHardcode;
import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp()
public class SpindexerHardcodeTest extends CommandOpMode {
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    Spindexer spindexer;
    TwoWheelShooter shooter;

    @Override
    public void initialize() {
        super.reset(); // Must put first
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        spindexer = new Spindexer(hardwareMap, false);
        shooter = new TwoWheelShooter(hardwareMap, TwoWheelShooter.RunMode.RawPower);

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            new ShootHardcode(spindexer, shooter, MotifEnums.Motif.GPP, true)
        );

        register(spindexer, shooter);
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
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.addData("Encoder postioin", spindexer.getEncoder().getAngle());
        telemetryM.addData("Spindexer position", spindexer.getCurrentAngle());
        telemetryM.update(telemetry);
        graphM.update();
    }
}