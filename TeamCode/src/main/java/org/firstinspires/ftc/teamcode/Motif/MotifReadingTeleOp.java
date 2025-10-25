package org.firstinspires.ftc.teamcode.Motif;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.util.StringTokenizer;

@TeleOp(group = "Motif", name = "Read")
public class MotifReadingTeleOp extends OpMode {
    String fileName = "Vision/motif_value.txt";
    File file;
    int number = -1;
    MotifEnums.Motif pattern;
    @Override
    public void init() {
        file = new File(Environment.getExternalStorageDirectory(),  fileName);
        if(file.exists()){
            String s = ReadWriteFile.readFile(file);
            number = s.equals("[21]") ? 21 : s.equals("[22]") ? 22 : s.equals("[23]") ? 23 : 0;
        }
        else{
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        pattern = number == 21 ? MotifEnums.Motif.GPP : number == 22 ? MotifEnums.Motif.PGP : number == 23 ? MotifEnums.Motif.PPG : MotifEnums.Motif.NONE;
    }

    @Override
    public void loop() {
        telemetry.addData("Motif ID", number);
        telemetry.addData("Motif Pattern", pattern);
        telemetry.update();
    }
}
