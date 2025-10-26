package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;


public class Timer {
    ElapsedTime elapsedTime;
    double previousTime;
    double currentTime;
    double deltaTime;

    public Timer(){
        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }

    public double getDeltaTime(){
        return deltaTime;
    }

    public double getPreviousTime(){
        return previousTime;
    }

    public double updateTime(){
        currentTime = elapsedTime.time();
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        return currentTime;
    }


    public void updatePreviousTime(){
        previousTime = currentTime;
    }

    public void restart(){
        elapsedTime = new ElapsedTime();
        previousTime = 0;
        currentTime = 0;
        deltaTime = 0;
        elapsedTime.startTime();
    }

}