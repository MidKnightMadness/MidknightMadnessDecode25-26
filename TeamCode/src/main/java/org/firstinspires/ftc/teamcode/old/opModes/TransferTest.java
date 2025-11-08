package org.firstinspires.ftc.teamcode.old.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.SortedSet;

// Individual motor test
// Also checks to see if configurable works
@Disabled
@Configurable
@TeleOp(group="Motors")
public class TransferTest extends OpMode {
    public static double motorPowerSwitchTime = 100;
    public static double motorPowerInitial = 1;
    public static double motorPower = 0.3;
    public static String motorName = "left";

    private TelemetryManager panelsTelemetry;
    private DcMotorEx motor;
    private SortedSet<String> allMotorEx, allMotor;
    private Timer timer;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        allMotor = hardwareMap.getAllNames(DcMotor.class);
        allMotorEx = hardwareMap.getAllNames(DcMotorEx.class);
        timer = new Timer();
    }

    @Override
    public void start() {
        timer.restart();
    }

    @Override
    public void loop() {
        if (timer.getTime() < motorPowerSwitchTime) {
            motor.setPower(motorPowerInitial);
        } else {
            motor.setPower(motorPower);
        }
        motor.setPower(motorPower);
        panelsTelemetry.addData("Motor name", motorName);
        panelsTelemetry.addData("All motors", allMotor);
        panelsTelemetry.addData("All motors (ex)", allMotorEx);
        panelsTelemetry.update(telemetry);
    }
}
