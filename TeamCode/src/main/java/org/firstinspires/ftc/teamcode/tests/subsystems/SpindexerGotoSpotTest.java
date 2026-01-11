package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerGotoSpot;
import org.firstinspires.ftc.teamcode.game.SpindexerSpot;
import org.firstinspires.ftc.teamcode.game.SpotType;
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
    Button nearestIntakeSpotButton;
    Spindexer spindexer;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    int targetSpot = 0;
    int nearestIntakeSpot = 0;

    @Override
    public void initialize() {
        super.reset();
//        CommandScheduler.getInstance().setBulkReading(
//                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
//        );

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap, false);
        spindexer.initAngle(); // would put this later but oh well
        gp1 = new GamepadEx(gamepad1);

        intakeSpot0Button = gp1.getGamepadButton(GamepadKeys.Button.X);
        intakeSpot1Button = gp1.getGamepadButton(GamepadKeys.Button.Y);
        intakeSpot2Button = gp1.getGamepadButton(GamepadKeys.Button.B);
        outakeSpot0Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
        outakeSpot1Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP);
        outakeSpot2Button = gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
        nearestIntakeSpotButton = gp1.getGamepadButton(GamepadKeys.Button.A);


        intakeSpot0Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 0),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.INTAKE, runMode, finishedTimeThreshold)
        ));
        intakeSpot1Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 1),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT1, SpotType.INTAKE, runMode, finishedTimeThreshold)
        ));
        intakeSpot2Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 2),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT2, SpotType.INTAKE, runMode, finishedTimeThreshold)
        ));

        outakeSpot0Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 0),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT0, SpotType.OUTTAKE, runMode, finishedTimeThreshold)
        ));
        outakeSpot1Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 1),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT1, SpotType.OUTTAKE, runMode, finishedTimeThreshold)
        ));
        outakeSpot2Button.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> targetSpot = 2),
                new SpindexerGotoSpot(spindexer, SpindexerSpot.SPOT2, SpotType.OUTTAKE, runMode, finishedTimeThreshold)
        ));

        nearestIntakeSpotButton.whenPressed((new SequentialCommandGroup(
                        new InstantCommand(() -> nearestIntakeSpot = spindexer.getNearestSpot(spindexer.getCurrentAngle(), SpotType.INTAKE).getIndex()),
                        new InstantCommand(() -> targetSpot = nearestIntakeSpot),
                        new SpindexerGotoSpot(spindexer, SpindexerSpot.fromIndex(nearestIntakeSpot), SpotType.INTAKE, runMode, finishedTimeThreshold
        ))));


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
//        addDataTelemetryGraph("Power", spindexer.getTurner().power);
//        telemetryM.addData("Test", spindexer.test);
//        telemetryM.addData("Positive power count", spindexer.getTurner().positivePowerCount);
//        telemetryM.addData("Power change count", spindexer.getTurner().powerChangeCount);
//        telemetryM.addData("Set call count", spindexer.getTurner().setCount);
//        telemetryM.addData("Revolutions", spindexer.getEncoder().getRevolutions());
////        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngle());
//        telemetryM.addData("Target Spot", targetSpot);
//        telemetryM.addData("Ball Colors", spindexer.getBallColors());
//        telemetryM.addData("Loop time (ms)", timer.getDeltaTime());
//        telemetryM.update(telemetry);
        telemetry.addData("Spot farthest from current angle", spindexer.farthestFromAngle(spindexer.getCurrentAngle(), SpotType.OUTTAKE));
        telemetry.addData("Current Angle", spindexer.getCurrentAngle());
        telemetry.addData("Error", spindexer.getTurner().error);
        telemetry.addData("Error2", spindexer.getTurner2().error);
        telemetry.addData("Revolutions", spindexer.getEncoder().getRevolutions());
        telemetry.addData("Spindexer Target Power1", spindexer.getTurner().power);
        telemetry.addData("Spindexer Target Power2", spindexer.getTurner2().power);
        telemetry.addData("Ball Colors", spindexer.getBallColors());
        telemetry.addData("Loop time (ms)", timer.getDeltaTime());

        telemetry.update();
        graphM.update();
    }
}