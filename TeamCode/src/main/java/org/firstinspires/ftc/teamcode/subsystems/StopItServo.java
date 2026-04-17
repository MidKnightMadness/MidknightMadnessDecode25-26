package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Configurable
public class StopItServo extends SubsystemBase {
    public static double activePosition = 0.37;
    public static double inactivePosition = 0.717;
    public static double initPos = inactivePosition;
    public static double cachingTolerance = 0.01;
    public static Servo.Direction dir = Servo.Direction.FORWARD;
    Servo stopItServo;

    public StopItServo(HardwareMap hardwareMap, boolean setStart){
        stopItServo = hardwareMap.get(Servo.class, ConfigNames.stopItServo);
        if(setStart) {
            stopItServo.setPosition(initPos);
        }
    }

    public void setActivePosition(){
        setPosition(activePosition);//min pose
    }
    public void setInactivePosition(){
        setPosition(inactivePosition);
    }

    public void setCustomPos(double pos){
        setPosition(pos);
    }

    public Servo getServo(){
        return stopItServo;
    }


    public void setPosition(double position){
        if ((Math.abs(position - stopItServo.getPosition()) > cachingTolerance) || (position == 0 && stopItServo.getPosition() != 0)) {
            stopItServo.setPosition(position);
        }
    }
}
