package org.firstinspires.ftc.teamcode.Benchmarking;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

// Measures loop times
public class BenchmarkUpdateRate extends OpMode {
    Timer timer;
    ArrayList<Long> loopTimes;
    String loopTimeFile = "loop_times.csv";

    @Override
    public void init() {
        timer = new Timer();
        loopTimes = new ArrayList<Long>();
    }

    @Override
    public void loop() {
        if (loopTimes.size() < 1000){
            loopTimes.add(timer.getLastUpdateTime());
        }
        if (loopTimes.size() == 1000) {
            try (FileWriter writer = new FileWriter(loopTimeFile)) {
                String writeString = loopTimes.toString();
                writer.write(writeString.substring(1, writeString.length() - 1));
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
