
package org.firstinspires.ftc.teamcode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * goBILDA Laser Distance Sensor Example (Analog Mode)
 *
 * Reads the analog output (0–3.3V) of the Dual-Mode Laser Distance Sensor
 * and converts it linearly to distance (0–1000 mm).
 *
 * 0.0V →   0 mm
 * 3.3V → 1000 mm
 *
 * Wiring/Config:
 * - Connect the sensor’s analog signal to a Hub Analog port.
 * - Name the device "laserAnalog" in Robot Configuration.
 *
 * Display:
 * - Driver Station telemetry shows voltage and distance (mm).
 */
@TeleOp(name = "laserAnalog")
public class GobildaDistanceTest extends LinearOpMode {
    private AnalogInput laserAnalog;

    // Sensor scale: 3.3V corresponds to ~1000 mm
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    @Override
    public void runOpMode() {
        // Map the analog device from the hardware configuration
        laserAnalog = hardwareMap.get(AnalogInput.class, "laserAnalogInput");

        // Wait for PLAY
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read sensor voltage (0.0–3.3V)
            double volts = laserAnalog.getVoltage();

            // Convert voltage to distance in millimeters (linear mapping)
            double distanceMM = (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

            // Telemetry
            telemetry.addData("Voltage (V)", "%.3f", volts);
            telemetry.addData("Distance (mm)", "%.1f", distanceMM);
            telemetry.update();
        }
    }
}
