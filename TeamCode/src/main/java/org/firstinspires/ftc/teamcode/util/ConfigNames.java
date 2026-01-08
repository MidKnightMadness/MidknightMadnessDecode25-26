package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class ConfigNames {
    public static String turner = "spindexerServo";
    public static String turner2 = "spindexerServo2";
    public static String turnerEncoder = "FR";
    public static String imu = "imu";
    public static String pinpoint1 = "pinpoint1";
    //TODO: MODIFY driver hub configs to use pinpoint 1
    public static String pinpoint2 = "pinpoint2";
    public static String limelight = "limelight";
    public static String lowFlywheelMotor = "lowFlywheelMotor";
    public static String highFlywheelMotor = "highFlywheelMotor";
    public static String intakeMotor = "intakeMotor";
    public static String FL = "FL";
    public static String FR = "FR";
    public static String BL = "BL";
    public static String BR = "BR";
    public static String rampServo = "rampServo";
    public static String intakeColorLeft = "intakeLeftColor";
    public static String intakeColorRight = "intakeRightColor";
}
