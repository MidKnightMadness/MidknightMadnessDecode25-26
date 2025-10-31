package org.firstinspires.ftc.teamcode.motif;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.Map;

@TeleOp(group = "Motif", name = "Read")
public class MotifReadingTeleOp extends OpMode {
    String fileName = "Vision/motif_value.txt";
    File file;
    String id;
    MotifEnums.Motif pattern;

    Map<String, MotifEnums.Motif> idMap = Map.of(
        "[21]", MotifEnums.Motif.GPP,
        "[22]", MotifEnums.Motif.PGP,
        "[23]", MotifEnums.Motif.PPG
    );


    @Override
    public void init() {
        file = new File(Environment.getExternalStorageDirectory(),  fileName);


        if(file.exists()){
            id = ReadWriteFile.readFile(file);
        }
        else{
            telemetry.addData("File does not exist", file.getAbsolutePath());
        }
        pattern = idMap.getOrDefault(id, MotifEnums.Motif.NONE);
    }

    @Override
    public void loop() {
        telemetry.addData("Motif ID", id);
        telemetry.addData("Motif Pattern", pattern);
        telemetry.update();
    }
}
