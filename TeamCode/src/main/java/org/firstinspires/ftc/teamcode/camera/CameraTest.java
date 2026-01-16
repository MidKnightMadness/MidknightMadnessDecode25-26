package org.firstinspires.ftc.teamcode.camera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.setServoCam;

import java.util.List;
@TeleOp
public class CameraTest extends OpMode {
    Limelight3A limelight;
    LLResult result;
    List<LLResultTypes.DetectorResult> detections;
    Servo cam;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(400); //default 100
        limelight.start();
        limelight.pipelineSwitch(0);
        cam = hardwareMap.get(Servo.class, "camServo");
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
        for (LLResultTypes.DetectorResult detection : detections) {
            String className = detection.getClassName(); // What was detected
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }
        if(detections.isEmpty()){
            telemetry.addData("Limelight", "No Detection");
        }
        telemetry.update();
        if(gamepad1.aWasPressed()){
            setServoCam.setCam(cam, 0.5);
        }
        if(gamepad1.bWasPressed()){
            setServoCam.setCam(cam, 1);
        }
        if(gamepad1.yWasPressed()){
            setServoCam.setCam(cam, 0.25);
        }
        if(gamepad1.xWasPressed()){
            setServoCam.setCam(cam, 0);
        }
    }
}
