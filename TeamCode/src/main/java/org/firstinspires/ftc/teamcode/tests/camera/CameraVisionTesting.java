package org.firstinspires.ftc.teamcode.tests.camera;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@TeleOp(name = "CameraVisionTesting", group = "Camera")
public class CameraVisionTesting extends OpMode {
    Limelight3A limelight;
    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(3);
        limelight.start();

    }

    @Override
    public void loop() {

    }
}
