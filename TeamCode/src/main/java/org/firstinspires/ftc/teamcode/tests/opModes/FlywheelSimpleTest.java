package org.firstinspires.ftc.teamcode.tests.opModes;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp
public class FlywheelSimpleTest extends OpMode {
    CRServo servo;
    DcMotorEx low;
    DcMotorEx high;
    TelemetryManager telemetryM;

    // Far: low 1, high 1
    // Close: low 0.67, high 1

    public static double lowPower = 1;
    public static double highPower = 1;

    @Override
    public void init() {
        low = hardwareMap.get(DcMotorEx.class, ConfigNames.lowFlywheelMotor);
        high = hardwareMap.get(DcMotorEx.class, ConfigNames.highFlywheelMotor);
        servo = hardwareMap.get(CRServo.class, ConfigNames.turner);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        low.setPower(lowPower);
        high.setPower(highPower);
        servo.setPower(-0.15);

        telemetryM.addData("low velocity", low.getVelocity());
        telemetryM.addData("high velocity", high.getVelocity());
        telemetryM.update(telemetry);
    }
}
