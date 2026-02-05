package org.firstinspires.ftc.teamcode.main.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand3;
import org.firstinspires.ftc.teamcode.game.BallColor;
import org.firstinspires.ftc.teamcode.tests.camera.CamCommand;
import org.firstinspires.ftc.teamcode.commands.intake.AutoIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pathing.SchedulePathTo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ConfigNames;
@Config
@Configurable

@Autonomous(name = "Camtestautomaybe", group = "Test")
public class CamAutoTestTemporaryDeleteLaterMaybe extends BaseAuto {
    int limelightPipelineStart = 3;//CHANGE THIS
    public static Pose startPose = new Pose(88, 8, Math.toRadians(270));//CHANGE THIS
    CamCommand cam;
    @Override
    protected Command preMotifSequence(){
        cam = new CamCommand(limelight, follower);
        return cam;
    }

    @Override
    protected boolean isVisionComplete(){
        return cam.isFinished();
    }


    @Override
    protected Command postMotifSequence() {
        //CamCommand cam = new CamCommand(limelight, follower);

        return// new SequentialCommandGroup(
                /*
                //runs camera and movementing until it finds balls
                        new RepeatCommand(cam),//does the camera thing
                        new RepeatCommand(strafeLeftRight(8)),//moves around so you can detect balls
                        new WaitUntilCommand(() -> !cam.finalBallList.isEmpty())

*/
                //when balls are found you go to them
                buildBallPathSequence(cam);
        //);
    }
    /*
    @Override
    protected Command postMotifSequence() {
        CamCommand cam = new CamCommand(limelight, follower);

        return new SequentialCommandGroup(
                //runs camera and movementing until it finds balls
                new ParallelRaceGroup(
                        new RepeatCommand(cam),//does the camera thing
                        new RepeatCommand(strafeLeftRight(8)),//moves around so you can detect balls
                        new WaitUntilCommand(() -> !cam.finalBallList.isEmpty())
                ),

                //when balls are found you go to them
                buildBallPathSequence(cam)
        );
    }
    */
    /*
    @Override//this one is chatgpt
    protected Command postMotifSequence() {
        CamCommand cam = new CamCommand(limelight, follower);

        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new RepeatCommand(cam),
                        new RepeatCommand(strafeLeftRight(8)),
                        new WaitUntilCommand(() -> cam.finalBallList != null && !cam.finalBallList.isEmpty())
                ),

                new InstantCommand(() -> {
                    new FollowPathCommand(
                            follower,
                            cam.getList(follower)
                    ).schedule();
                })
        );
    }
*/

    private Command buildBallPathSequence(CamCommand camera) {
        /*
        ParallelCommandGroup parallel = new ParallelCommandGroup();
        //parallel.addCommands(new AutoIntakeCommand3(spindexer, intake, 0.5, 0.5, false, hardwareMap));
        if(camera.finalBallList != null && camera.finalBallList.size() > 0) {
            parallel.addCommands(new FollowPathCommand(follower, camera.getList(follower)));
        }
        */
        if(camera.getList(follower)!= null && camera.getList(follower).getPath(0)!= null) {
            return new FollowPathCommand(follower, camera.getList(follower).getPath(0));
        } else{
            return new InstantCommand();
        }
    }
    private Command strafeLeft(double distance){
        Pose botPose = follower.getPose();
        Pose leftPose = new Pose(botPose.getX(), botPose.getY()+distance, botPose.getHeading());
        return new FollowPathCommand(follower, leftPose, 0.5);//this moves left

    }

    @Override
    protected void initializeMechanisms() {
        intake = new Intake(hardwareMap, Intake.RunMode.RawPower);//init intak
        spindexer = new Spindexer(hardwareMap, true).setBallColors(new BallColor[]{BallColor.NONE, BallColor.NONE, BallColor.NONE}).initAngle();//init spindexer
    }

    @Override
    protected void setupVision() {
        limelight = hardwareMap.get(Limelight3A.class, ConfigNames.limelight);//init limelight
        limelight.pipelineSwitch(limelightPipelineStart);
        limelight.start();
    }

    @Override
    protected Pose getStartPose(){
        return startPose;
    }

    @Override
    protected void updateTelemetry(){
        if(cam != null) {
            telemetry.addData("camcommand finished", cam.isFinished());
        }
        telemetry.addData("Robot position", follower.getPose());
        telemetry.update();
        if(cam.getList(follower) == null || cam.getList(follower).getPath(0)== null) {
            telemetry.addData("first one is null", "null");
        }
        else{
            telemetry.addData("start", cam.getList(follower).getPath(0).getPose(0));
            telemetry.addData("final", cam.getList(follower).getPath(0).getPose(0));
            telemetry.addData("closest", cam.getMinBallPose().getX());
            telemetry.addData("closest", cam.getMinBallPose().getX());
            for(Pose pose : cam.getFinalBallList()){
                telemetry.addData("pose X", pose.getX());
                telemetry.addData("pose y", pose.getY());
            }
            telemetry.addData("homography pose", cam.getHomographyPose());
        }
        follower.update();
    }
}
