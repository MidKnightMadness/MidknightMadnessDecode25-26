

package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.robotDrive.WheelControl;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
@Autonomous(name = "Park Forward Auto", group = "Competition")
public class ParkForwardAuto extends CommandOpMode {
    public static double motifDetectionTimeMs = 5000;
    public static double driveForwardTime = 400;
    int startPipeline = 1;
    public static Pose startPose = new Pose(122, 120, Math.toRadians(-135));
    WheelControl wheelControl;
    Timer timer;
    double driveTime = 3;


    @Override
    public void initialize() {
        timer = new Timer();
        wheelControl = new WheelControl(hardwareMap);
    }

    @Override
    public void run(){
        super.run();
        if (timer.getTime() < driveForwardTime) {
            wheelControl.driveRelative (1, 0, 0, 1);
        } else {
            wheelControl.stop();
        }
    }
}




