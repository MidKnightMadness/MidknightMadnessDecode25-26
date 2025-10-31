package org.firstinspires.ftc.teamcode.tests.sensors;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.colors.ColorClassifier;
import org.firstinspires.ftc.teamcode.colors.Threshold;
import org.firstinspires.ftc.teamcode.hardware.BallDetector;
import org.firstinspires.ftc.teamcode.colors.BallColor;
import org.firstinspires.ftc.teamcode.hardware.SensorColorEx;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(group="Sensors")
public class BallDetectorTest extends OpMode {
    BallColor detected;
    BallDetector ballDetector;
    TelemetryManager telemetryM;
    GraphManager graphM;
    Timer timer;
    ColorClassifier<BallColor> classifier;

    @Override
    public void init() {
        classifier = classifier.toBuilder()
                .add(BallColor.GREEN,
                        Threshold.any(),
                        Threshold.of(100, 200),
                        Threshold.any()
                )
                .add(BallColor.PURPLE,
                        Threshold.any(),
                        Threshold.of(100, 200),
                        Threshold.any()
                )
                .setDefault(BallColor.NONE)
                .build();
        PanelsDrawing.init();
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        ballDetector = new BallDetector(hardwareMap, ConfigNames.inColorSensor);
        SensorColorEx sensor = new SensorColorEx(hardwareMap, "hello");
    }

    @Override
    public void init_loop() {
        updateTelemetry();
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
        telemetryM.addData("HSV", ballDetector.readRawColor());
        telemetryM.addData("Detected color", ballDetector.readColor());
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
        graphM.update();
    }
}