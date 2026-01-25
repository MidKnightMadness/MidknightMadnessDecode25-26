package org.firstinspires.ftc.teamcode.tests.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Ranger Distance Sensor Test", group="Test")
public class RangerDistanceSensorTest extends LinearOpMode {

    private AnalogInput rangerAnalog;
    private DigitalChannel rangerDetect;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            rangerAnalog = hardwareMap.get(AnalogInput.class, "rangerAnalog");
        } catch (Exception e) {
            rangerAnalog = null;
        }

        try {
            rangerDetect = hardwareMap.get(DigitalChannel.class, "rangerDetect");
            rangerDetect.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            rangerDetect = null;
        }

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.clear();

            if (rangerAnalog != null) {

                double voltage = rangerAnalog.getVoltage();

                /*
                 * Placeholder conversion:
                 * You MUST calibrate this yourself.
                 * This assumes roughly:
                 * 0.5V = very close
                 * 3.0V = far
                 */

                double estimatedDistanceCM =
                        (voltage - 0.5) * (200.0 / 2.5);

                telemetry.addData("Voltage", "%.3f V", voltage);
                telemetry.addData("Est Distance", "%.1f cm", estimatedDistanceCM);
            }

            if (rangerDetect != null) {
                telemetry.addData("Detect", rangerDetect.getState() ? "HIGH" : "LOW");
            }

            telemetry.addData("Loop ms", getRuntime() * 1000);

            telemetry.update();
        }
    }
}