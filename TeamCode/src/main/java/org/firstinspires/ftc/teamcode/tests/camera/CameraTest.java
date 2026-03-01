package org.firstinspires.ftc.teamcode.tests.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import java.util.List;
@TeleOp
@Disabled
public class CameraTest extends OpMode {
    Limelight3A limelight;
    LLResult result;
    List<LLResultTypes.DetectorResult> detections;
    Servo cam;

    ArrayList<Double> coordX;
    ArrayList<Double> coordY;
    ArrayList<Double> distances;
    double min;
    int index;
    double closeX;
    double closeY;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400); //default 100
        limelight.start();
        limelight.pipelineSwitch(0);
        cam = hardwareMap.get(Servo.class, "camServo");
        coordX = new ArrayList<>();
        coordY = new ArrayList<>();
        distances = new ArrayList<>();
        min = 0;
        index = 0;
        closeX = 0;
        closeY = 0;
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


        if(detections.isEmpty()|| result == null){
            telemetry.addData("Limelight", "No Detection");
            telemetry.update();
            coordX.clear();
            coordY.clear();
            distances.clear();
            index = 0;
            closeX = 0;
            closeY = 0;
        }
        else{
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
                //run it through homography, not made yet
                //telemetry.addData(className, "at (" + coordinateX + ", " + coordinateY + ") coordinates");
                //coordX.add(coordinateX);
                //coordY.add(coordinateY);
            }
            for (int i = 0; i < detections.size(); i++) {
                distances.add(distance(coordX.get(i), coordY.get(i)));
            }
            min = distances.get(0);
            for (int i = 0; i < distances.size(); i++) {
                if (distances.get(i) < min) {
                    min = distances.get(i);
                    index = i;
                }
            }
            closeX = coordX.get(index);
            closeY = coordY.get(index);
            telemetry.addData("CloseX: ", closeX);
            telemetry.addData("CloseY: ", closeY);

            telemetry.update();
            index = 0;
            coordX.clear();
            coordY.clear();
            distances.clear();
        }
    }
    private static double distance(double x, double y){
        return Math.sqrt(x*x+y*y);
    }

}














