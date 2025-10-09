package org.firstinspires.ftc.teamcode.Util;

import android.icu.text.Transliterator;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;

@Configurable
public class PoseBuffer {
    ArrayList<Pose3D> list;
    static int BUFFER_SIZE = 10;
    Position previousAvgPose;
    public PoseBuffer(){
        list = new ArrayList<>();
        previousAvgPose = new Position();
    }


    public Position update(Pose3D pose){
        if(pose != null) {
            list.add(pose);
        }
        if(list.size() == BUFFER_SIZE){
            previousAvgPose = averagePoses();
        }
        return previousAvgPose;
    }

    private Position averagePoses(){
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;
        for(int i = 0; i < list.size(); i++){
            sumX += list.get(i).getPosition().x;
            sumY += list.get(i).getPosition().y;
            sumZ += list.get(i).getPosition().z;
        }

        double avgX = sumX / BUFFER_SIZE;
        double avgY = sumY / BUFFER_SIZE;
        double avgZ = sumZ / BUFFER_SIZE;
        list.clear();
        return new Position(DistanceUnit.INCH, avgX, avgY, avgZ, 0);
    }

}

