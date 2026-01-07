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
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.spindexer.SpindexerMotifSequence;
import org.firstinspires.ftc.teamcode.hardware.CRServoEx2;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Angle;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.util.Timer;

@Configurable
@TeleOp(name="SpindexerMotifTest", group = "Spindexer")
public class SpindexerMotifTest extends CommandOpMode {
    public static CRServoEx2.RunMode runMode = CRServoEx2.RunMode.OptimizedPositionalControl;
    public static BallColor[] ballColors = { BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
    public static Angle customAngle = Angle.fromDegrees(60);
    public static double finishedTimeThreshold = 100;

    Button spot0Button, spot1Button, spot2Button, customAngleButton;
    Spindexer spindexer;
    GamepadEx gp1;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    int targetSpot = 0;
    MotifEnums.Motif motif = MotifEnums.Motif.PGP;

    @Override
    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().setBulkReading(
                hardwareMap, LynxModule.BulkCachingMode.MANUAL // Scheduler will clean cache for you
        );

        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        spindexer = new Spindexer(hardwareMap, false)
                .initAngle()
                .setBallColors(ballColors);
        gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SequentialCommandGroup(
//                new InstantCommand(() -> targetSpot = 0),

                new SpindexerMotifSequence(
                        spindexer, motif,
                        runMode, finishedTimeThreshold
                )
        ));

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
//        addDataTelemetryGraph("Power", spindexer.getTurner().power);
        if(spindexer.getSequence() != null) {
            telemetry.addData("Sequence", spindexer.getSequence()[0] + " " + spindexer.getSequence()[1] + " " + spindexer.getSequence()[2]);
        }
//        telemetryM.addData("Sub angle thing", spindexer.test);
        telemetry.addData("Revolutions", spindexer.getEncoder().getRevolutions());
//        addDataTelemetryGraph("Raw Angle", spindexer.getEncoder().getAngle());
        telemetry.addData("Target Spot", targetSpot);
        telemetry.addData("Ball Colors", spindexer.getBallColors());
        telemetry.addData("Loop time (ms)", timer.getDeltaTime());
        telemetry.update();
//        graphM.update();
    }
}