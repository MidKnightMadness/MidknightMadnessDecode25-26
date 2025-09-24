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
    public static double motorNum = 1;
    //1 = BR, 2 = BL, 3 = FL, 4 = FR
    int loop = 1;
    TelemetryManager panelsTelemetry;
    DcMotorEx motor;
    String motorName = "FL";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        loop += 1;
        motorName = motorNum == 1 ? "BR" : motorNum == 2 ? "BL" : motorNum == 3 ? "FL" : "FR";
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setPower(0.5);
        panelsTelemetry.addData("Motor name",  motorName);
        panelsTelemetry.addData("Loop",  loop);
        panelsTelemetry.update();
    }
}
