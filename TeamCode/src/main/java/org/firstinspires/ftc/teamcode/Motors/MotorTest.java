package org.firstinspires.ftc.teamcode.Motors;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Individual motor test
// Also checks to see if configurable works
@Configurable
@TeleOp(group="Motors")
public class MotorTest extends OpMode {
    public static String motorName = "BR";
    TelemetryManager panelsTelemetry;
    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        motor.setPower(0.5);
        panelsTelemetry.addData("Motor name",  motorName);
        panelsTelemetry.update();
    }
}
