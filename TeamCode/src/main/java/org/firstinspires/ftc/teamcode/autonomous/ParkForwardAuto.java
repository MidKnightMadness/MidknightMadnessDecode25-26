

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CommandOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.motorTesting.WheelControl;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@Configurable
@Autonomous(name = "Park Forward Auto", group = "Competition")
public class ParkForwardAuto extends CommandOpMode {
    public static double motifDetectionTimeMs = 5000;
    int startPipeline = 1;
    public static Pose startPose = new Pose(122, 120, Math.toRadians(-135));
    WheelControl wheelControl;

//    public static Pose motifDetectionPose = new Pose(88, 112, Math.toRadians(115));
//    public static Pose shootPose = new Pose(85, 85, Math.toRadians(40));
//    public static Pose leavePose = new Pose(86, 65, Math.toRadians(-90));
//    PathChain toMotifPath;
//    PathChain toShootingPath;
//    PathChain leaveBasePath;
//    MotifEnums.Motif motifPattern;
//    MotifWriteCommand motifCommand;

//    ShootSide shootSide = ShootSide.LEFT;
//    Pose currentPose;
//
//    double speed;
//    double acc;
//
//    public static long waitTime = 3000;
//    public static double pathDistThresholdMin = 3;
//    public static double headingError = 0.3;

    Timer timer;
    double driveTime = 3;


    @Override
    public void initialize() {
        timer = new Timer();
        wheelControl = new WheelControl(hardwareMap);
    }

    @Override
    public void first() {
        timer.restart();
    }

    @Override
    public void run(){
        super.run();
        if (timer.getTime() < 400) {
            wheelControl.drive_relative(1, 0, 0, 1);
        } else {
            wheelControl.stop();
        }
//        } else if (timer.getTime() < 3000) {
//            wheelControl.drive_relative(0, 0, 1, 1);
//        }
    }
}




