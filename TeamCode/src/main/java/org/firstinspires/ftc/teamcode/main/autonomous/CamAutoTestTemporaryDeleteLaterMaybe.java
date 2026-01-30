package org.firstinspires.ftc.teamcode.main.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;

@Autonomous(name = "Camtestautomaybe", group = "Test")
public class CamAutoTestTemporaryDeleteLaterMaybe extends BaseAuto {

    // Key positions
    public static Pose startPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose shootPose = new Pose(60, 17, Math.toRadians(295));
    public static Pose forwardPose = new Pose(56, 12, Math.toRadians(90));
    public static Pose parkPose = new Pose(58, 38, Math.toRadians(180));

    // Paths
    Path toShootPresets;
    Path toPark;

    @Override
    protected Pose getStartPose() {
        return startPose;
    }

    @Override
    protected void buildPaths() {
        // Drive from start → shoot
        toShootPresets = new Path(new BezierLine(forwardPose, shootPose));
        toShootPresets.setLinearHeadingInterpolation(forwardPose.getHeading(), shootPose.getHeading());

        // Drive from shoot → park
        toPark = new Path(new BezierLine(shootPose, parkPose));
        toPark.setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading());
    }

    @Override
    protected Command preMotifSequence() {
        // Nothing yet
        return null;
    }

    @Override
    protected Command postMotifSequence() {
        // Sequentially follow paths
        return new SequentialCommandGroup(
                new InstantCommand(()-> ),
                new ParallelCommandGroup(
                    new SchedulePathTo(follower, shootPose),
                    new AutoIntakeCommand(intake, spindexer, )
                )
        );
    }

    @Override
    protected void initializeMechanisms() {
        // No shooter yet
    }

    @Override
    protected void setupVision() {
        // No vision yet
    }
}
