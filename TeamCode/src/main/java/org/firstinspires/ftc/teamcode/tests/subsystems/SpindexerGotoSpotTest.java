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
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@TeleOp(group="Subsystems")
public class SpindexerGotoSpotTest extends CommandOpMode {
    public static CRServoEx2.RunMode runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    public static Angle customAngle = Angle.fromDegrees(60);
    public static double finishedTimeThreshold = 1000;

    Button intakeSpot0Button, intakeSpot1Button, intakeSpot2Button;
    Button outakeSpot0Button, outakeSpot1Button, outakeSpot2Button;
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
        spindexer.initAngle(); // would put this later but oh well
        gp1 = new GamepadEx(gamepad1);

        intakeSpot0Button = gp1.getGamepadButton(GamepadKeys.Button.A);
        intakeSpot1Button = gp1.getGamepadButton(GamepadKeys.Button.B);
        intakeSpot2Button = gp1.getGamepadButton(GamepadKeys.Button.X);
        outakeSpot0Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        outakeSpot1Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        outakeSpot2Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);

//        intakeSpot0Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 0),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.ZERO_INTAKE, runMode, finishedTimeThreshold)
//        ));
//        intakeSpot1Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 1),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.ONE_INTAKE, runMode, finishedTimeThreshold)
//        ));
//        intakeSpot2Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 2),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.TWO_INTAKE, runMode, finishedTimeThreshold)
//        ));
//        outakeSpot0Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 0),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.ZERO_OUTTAKE, runMode, finishedTimeThreshold)
//        ));
//        outakeSpot1Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 1),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.ONE_OUTTAKE, runMode, finishedTimeThreshold)
//        ));
//        outakeSpot2Button.whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 2),
//                new SpindexerGotoSpot(spindexer, SpindexerSpot.TWO_OUTTAKE, runMode, finishedTimeThreshold)
//        ));


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
//        telemetryM.addData("Test", spindexer.test);
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