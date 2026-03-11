package org.firstinspires.ftc.teamcode.templates;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.concurrent.TimeUnit;

@Disabled
@Configurable
@Autonomous
public class PanelsTemplate extends OpMode {
    TelemetryManager telemetryM;
    Timer timer;

    @Override
    public void init() {
        PanelsDrawing.init();
        timer = new Timer();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        updateTelemetry();
    }

    @Override
    public void start() {
        timer.restart();
    }

    @Override
    public void loop() {
        updateTelemetry();
    }

    public void addDataTelemetryGraph(String key, Number value) {
        telemetryM.addData(key, value);
    }

    public void updateTelemetry() {
        addDataTelemetryGraph("Loop time (ms)", timer.getDeltaTime(TimeUnit.MILLISECONDS));
        telemetryM.update(telemetry);
    }
}