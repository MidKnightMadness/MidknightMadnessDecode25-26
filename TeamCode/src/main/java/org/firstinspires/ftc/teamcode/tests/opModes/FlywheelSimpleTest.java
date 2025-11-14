package org.firstinspires.ftc.teamcode.tests.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
@TeleOp(name = "Flywheel Simple Test", group = "Flywheel")
public class FlywheelSimpleTest extends OpMode {
    CRServo servo;
    DcMotorEx low;
    DcMotorEx high;
    TelemetryManager telemetryM;

    // Far: low 1, high 1
    // Close: low 0.67, high 1

    public static double lowPower = 1;
    public static double highPower = 1;
    public static double dir = -1;
    public static double spindexerPower = 1;
    final double threeToOneGR = 2.89;
    final double fourToOneGR = 3.61;
    final double fiveToOneGR = 5.23;
    final double ticksPerRevolution = 28;
    double lowRPM = 0;
    double highRPM = 0;
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
        servo.setPower(dir * spindexerPower);

        lowRPM = low.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;
        highRPM = high.getVelocity() / ticksPerRevolution * 60 / threeToOneGR;

        telemetryM.addData("low velocity", low.getVelocity());
        telemetryM.addData("high velocity", high.getVelocity());
        telemetryM.addData("low rpm", lowRPM);
        telemetryM.addData("high velocity", highRPM);
        telemetryM.update(telemetry);
    }
}
