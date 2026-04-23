package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Configurable
public class PushUpServo extends SubsystemBase {
    public static double minPos = 0.5;
    public static double maxPos = 0.95;
    public static double initPos = minPos;
    public static double cachingTolerance = 0.001;
    public static Servo.Direction dir = Servo.Direction.FORWARD;
    Servo pushUpServo;

    public PushUpServo(HardwareMap hardwareMap){
        pushUpServo = hardwareMap.get(Servo.class, ConfigNames.pushUpServo);
//        pushUpServo.setDirection(dir);
        pushUpServo.setPosition(initPos);
    }
    public PushUpServo(HardwareMap hardwareMap, boolean setStart){
        pushUpServo = hardwareMap.get(Servo.class, ConfigNames.pushUpServo);
        if(setStart) {
            pushUpServo.setPosition(initPos);
        }
    }

    public void setDown(){
        setPosition(minPos);
    }
    public double getMinPos(){
        return minPos;
    }
    public double getMaxPos(){
        return maxPos;
    }

    public void setUp(){
        setPosition(maxPos);
    }

    public void setCustomPos(double pos){
        setPosition(Math.max(Math.min(pos, maxPos), minPos));
    }

    public Servo getServo(){
        return pushUpServo;
    }


    public void setPosition(double position){
        if ((Math.abs(position - pushUpServo.getPosition()) > cachingTolerance) || (position == 0 && pushUpServo.getPosition() != 0)) {
            pushUpServo.setPosition(position);
        }
    }
}
