package org.firstinspires.ftc.teamcode.tests;

import android.content.res.AssetManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Robot;

import java.io.IOException;
import java.io.InputStream;

@TeleOp(name = "Robot File Config Test")
public class ConfigTest extends OpMode {
    TelemetryManager telemetryM;
    boolean assetFound = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetryM.addLine("test");
        telemetryM.addData("HELLO", Robot.config.get("HELLO"));
        telemetryM.addData("THREE", Robot.config.get("THREE"));
        telemetryM.update(telemetry);
    }
}
