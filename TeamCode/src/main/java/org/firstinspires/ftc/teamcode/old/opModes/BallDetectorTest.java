package org.firstinspires.ftc.teamcode.old.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(group="Sensors")
@Disabled
public class BallDetectorTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
//    BallColor detected;
//    BallDetector ballDetector;
//    TelemetryManager telemetryM;
//    GraphManager graphM;
//    Timer timer;
//    ColorClassifier<BallColor> classifier;
//
//    @Override
//    public void init() {
//        classifier = classifier.toBuilder()
//                .add(BallColor.GREEN,
//                        Threshold.any(),
//                        Threshold.of(100, 200),
//                        Threshold.any()
//                )
//                .add(BallColor.PURPLE,
//                        Threshold.any(),
//                        Threshold.of(100, 200),
//                        Threshold.any()
//                )
//                .setDefault(BallColor.NONE)
//                .build();
//        PanelsDrawing.init();
//        timer = new Timer();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//        graphM = PanelsGraph.INSTANCE.getManager();
//        ballDetector = new BallDetector(hardwareMap, ConfigNames.inColorSensor);
//        SensorColorEx sensor = new SensorColorEx(hardwareMap, "hello");
//    }
//
//    @Override
//    public void init_loop() {
//        updateTelemetry();
//    }
//
//    @Override
//    public void loop() {
//        updateTelemetry();
//    }
//
//    public void addDataTelemetryGraph(String key, Number value) {
//        telemetryM.addData(key, value);
//        graphM.addData(key, value);
//    }
//
//    public void updateTelemetry() {
//        telemetryM.addData("HSV", ballDetector.readRawColor());
//        telemetryM.addData("RGB", ballDetector.readRawColor());
//        telemetryM.addData("Detected color", ballDetector.readColor());
//        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
//        telemetryM.update(telemetry);
//        graphM.update();
//    }
}