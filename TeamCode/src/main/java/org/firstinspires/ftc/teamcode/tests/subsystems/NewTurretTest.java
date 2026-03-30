package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

@TeleOp(name = "Turret Prototype Test", group = "Test")
@Config
@Configurable
public class NewTurretTest extends OpMode {
    public static double targetHeading = 0;
    TelemetryManager telemetryM;
    Turret turret;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = new Turret(hardwareMap, true);
    }

    @Override
    public void loop() {
        turret.setAngleOptimized(Angle.fromDegrees(targetHeading));

        telemetry.addData("Target Heading", targetHeading);
        telemetry.update();
    }


}