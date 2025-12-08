

package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MotifWriteCommand;
import org.firstinspires.ftc.teamcode.commands.ShootHardcode;
import org.firstinspires.ftc.teamcode.game.MotifEnums;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsBot;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
import org.firstinspires.ftc.teamcode.game.ShootSide;

@Config
@Configurable
@Autonomous(name = "3 Close Left", group = "Competition")
public class ThreeBallCloseLeftAuto extends ThreeBallCloseRightAuto {
    static {
          startPose = new Pose(144-118.7, 130, Math.toRadians(180 - 43));
          motifDetectionPose = new Pose(144 - 87, 94, Math.toRadians(180 - 100));
          shootPose = new Pose(144 - 87, 94, Math.toRadians(230+90));
          leavePose = new Pose(144 - 85, 67, Math.toRadians(180));
          shootSide = ShootSide.LEFT;
    }

}




