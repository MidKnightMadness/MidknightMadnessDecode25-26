package org.firstinspires.ftc.teamcode.camera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.setServoCam;

import java.util.ArrayList;

import java.util.List;

@Configurable
@Config
@TeleOp(name = "HomographyTest")
public class camerathingy extends OpMode {
    Limelight3A limelight;
    LLResult result;
    List<LLResultTypes.DetectorResult> detections;
    double minX;
    double minY;
    double minD;
    double coordinateX;
    double coordinateY;
    private static final double horizontalFOV = 54.5;
    private static final double verticalFOV = 42;
    private static final double camWidth = 1280;
    private static final double camHeight = 960;
    private static final double PPI = 96;
    private static final double hOffset = 0;
    private static final double vOffset = 0;
    private final double[][] H = {
            {5.828680, 1.841763, -3515.724491},
            {0.052028, 13.011093, -4470.243506},
            {0.000090, 0.003892, 1.000000}
    };

    private static double distance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
    private void processHomography(double x_degrees, double y_degrees) {//this takes degrees and sets coordinateX and Y to the homographied thing
        double x = (x_degrees / horizontalFOV) * camWidth + (camWidth / 2.0);
        double y = (y_degrees / verticalFOV) * camHeight + (camHeight / 2.0);

        double X_prime = H[0][0] * x + H[0][1] * y + H[0][2];
        double Y_prime = H[1][0] * x + H[1][1] * y + H[1][2];
        double W = H[2][0] * x + H[2][1] * y + H[2][2];

        double x_robot = X_prime / W / PPI + hOffset;
        double y_robot = Y_prime / W / PPI + vOffset;

        coordinateX = x_robot;
        coordinateY = y_robot;
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400); //default 100
        limelight.start();
        limelight.pipelineSwitch(0);
        minX = Double.MAX_VALUE;
        minY = Double.MAX_VALUE;
        minD = Double.MAX_VALUE;
    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();
        /*
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

         */
        detections = result.getDetectorResults();


        if (detections.isEmpty() || result == null) {
            telemetry.addData("Limelight", "No Detection");
            telemetry.update();
        } else {
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                //run it through homographyhgfghjkjhgfghjhgfd
                //telemetry.addData(className, "at (" + coordinateX + ", " + coordinateY + ") coordinates");
                processHomography(x, y);
                telemetry.addData(className, "at (" + coordinateX + ", " + coordinateY + ") coordinates. Distance: " + distance(coordinateX, coordinateY));
                if (distance(coordinateX, coordinateY) < minD) {
                    minX = coordinateX;
                    minY = coordinateY;
                    minD = distance(coordinateX, coordinateY);
                }
            }
            telemetry.addData("Closest ball: ", "(" + minX + ", " + minY + ")" + "distance: " + minD);
            minD = Double.MAX_VALUE;
            minX = Double.MAX_VALUE;
            minY = Double.MAX_VALUE;
            telemetry.update();

        }
    }
}















