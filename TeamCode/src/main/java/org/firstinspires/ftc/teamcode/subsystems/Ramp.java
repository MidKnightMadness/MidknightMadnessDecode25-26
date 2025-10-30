package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.util.ConfigNames;

@Config
@Configurable
public class Ramp extends SubsystemBase {
    ServoEx rampServo;
    public static double lowPos = 0;
    public static double restPos = 0.5;
    public static double leftBound = 0;
    public static double rightBound = 1;
    public static boolean reversed = false;

    public Ramp(HardwareMap hardwareMap){
        rampServo = new ServoEx(hardwareMap, ConfigNames.rampServo, leftBound, rightBound);
        rampServo.setInverted(reversed);
    }

    public void setLowerPos(){
        rampServo.set(lowPos);
    }

    public void setRestPos(){
        rampServo.set(restPos);
    }

    public boolean setPos(double pos){
        if(pos > lowPos && pos < restPos){
            rampServo.set(pos);
            return true;
        }
        return false;
    }

    public double getPos(){
        return rampServo.get();
    }

}
