//package org.firstinspires.ftc.teamcode.tests.opModes;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
//@TeleOp(name="Ranger Distance Sensor Test", group="Test")
//@Configurable
//public class RangerDistanceSensorTest extends LinearOpMode {
//
//    private AnalogInput rangerAnalog;
//    private DigitalChannel rangerDetect;
//
//    // cm -> inches conversion (preserves calibration accuracy)
//    private static final double CM_TO_IN = 1.0 / 2.54;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        boolean analogFound = true;
//        boolean digitalFound = true;
//
//        try {
//            rangerAnalog = hardwareMap.get(AnalogInput.class, "rangerAnalog");
//        } catch (Exception e) {
//            rangerAnalog = null;
//            analogFound = false;
//        }
//
//        try {
//            rangerDetect = hardwareMap.get(DigitalChannel.class, "rangerDetect");
//            rangerDetect.setMode(DigitalChannel.Mode.INPUT);
//        } catch (Exception e) {
//            rangerDetect = null;
//            digitalFound = false;
//        }
//
//        telemetry.addLine("Initialized");
//        telemetry.addData("Analog Sensor", analogFound ? "OK" : "NOT FOUND");
//        telemetry.addData("Digital Detect", digitalFound ? "OK" : "NOT FOUND");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            telemetry.clear();
//
//            if (rangerAnalog != null) {
//
//                double voltage = rangerAnalog.getVoltage();
//
//                /*
//                 * Placeholder conversion:
//                 * You MUST calibrate this yourself.
//                 * This assumes roughly:
//                 * 0.5V = very close
//                 * 3.0V = far
//                 */
//
//                double estimatedDistanceCM =
//                        (voltage - 0.093) * (200.0 / 2.5);
//
//                // convert to inches AFTER calculation to preserve accuracy
//                double estimatedDistanceIN = estimatedDistanceCM * CM_TO_IN;
//
//                telemetry.addData("Voltage", "%.3f V", voltage);
//                telemetry.addData("Estimated Distance", "%.2f in", estimatedDistanceIN);
//
//            } else {
//                telemetry.addLine("⚠ ANALOG SENSOR NOT FOUND");
//            }
//
//            if (rangerDetect != null) {
//                telemetry.addData(
//                        "Detect",
//                        rangerDetect.getState() ? "HIGH" : "LOW"
//                );
//            } // maybe use .getState to determine a detection? depends on accuracy needed and purpose
//            else {
//                telemetry.addLine("⚠ DIGITAL DETECT NOT FOUND");
//            }
//
//            telemetry.addData("Loop ms", "%.1f", getRuntime() * 1000);
//
//            telemetry.update();
//        }
//    }
//}
package org.firstinspires.ftc.teamcode.tests.sensors;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RangerMode;
import org.firstinspires.ftc.teamcode.hardware.SwyftRanger;
import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name="Ranger Proximity Sensor Test (Digital)", group="Test")
public class RangerDistanceSensorTest extends LinearOpMode {

    SwyftRanger ranger;

    @Override
    public void runOpMode() {
        ranger = new SwyftRanger(hardwareMap, ConfigNames.intakeDist1, RangerMode.DEG15);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double distance = ranger.getDistance();
            // telemetry.addData("Inch 15DEG 0-1 Mode: ", (ranger.getVoltage()*32.5)-2.6);
            // telemetry.addData("Inch 20DEG 0-0 Mode: ", (ranger.getVoltage()*48.7)-4.9);
            // telemetry.addData("Inch 27DEG 1-0 Mode: ", (ranger.getVoltage()*78.1)-10.2);
            telemetry.addData("Distance", distance);

            telemetry.update();
        }
    }
}