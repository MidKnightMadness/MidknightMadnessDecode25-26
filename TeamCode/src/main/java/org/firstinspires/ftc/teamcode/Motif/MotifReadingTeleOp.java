package org.firstinspires.ftc.teamcode.Motif;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.util.StringTokenizer;

@TeleOp(group = "Motif", name = "Read")
public class MotifReadingTeleOp extends OpMode {
    String fileName = "motif_value.txt";
    File file;
    int motifPattern = -1;
    @Override
    public void init() {

        file = AppUtil.getInstance().getSettingsFile(fileName);
        motifPattern = Integer.parseInt(ReadWriteFile.readFile(file).trim());

    }

    @Override
    public void loop() {
        telemetry.addData("Motif Pattern", motifPattern);
        telemetry.update();
    }
}
