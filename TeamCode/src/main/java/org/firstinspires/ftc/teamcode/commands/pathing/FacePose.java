package org.firstinspires.ftc.teamcode.commands.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.util.ExtraFns;

public class FacePose extends SequentialCommandGroup {

    public FacePose(Follower follower, Pose pose) {
        addCommands(new TurnToCommand(
                follower,
                ExtraFns.getAngle(follower.getPose(), pose).toRadians()
        ));
    }

}
