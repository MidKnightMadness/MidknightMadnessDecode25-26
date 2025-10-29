package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.motif.MotifEnums;
import org.firstinspires.ftc.teamcode.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.TwoWheelShooter;
import org.firstinspires.ftc.teamcode.util.ShootSide;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.File;
import java.io.FileWriter;
import java.util.Map;

public class BaseAuto extends CommandOpMode {
    Limelight3A limelight;
    TelemetryManager telemetryManager;
    GraphManager graphManager;
    Follower follower;
    Timer timer;
    int startPipeline = 1;

    Pose startPose = getStartPose();
    boolean finishedWriting = false;
    String fileName = "motif_value.txt";
    String directoryName = "Vision";
    FileWriter fileWriter;
    File file;
    Spindexer spindexer;
    Ramp ramp;
    TwoWheelShooter shooter;
    MotifEnums.Motif motifPattern = MotifEnums.Motif.NONE;
    Map<Integer, MotifEnums.Motif> idMap = Map.of(
            21, MotifEnums.Motif.GPP,
            22, MotifEnums.Motif.PGP,
            23, MotifEnums.Motif.PPG
    );
    ShootSide shootSide = ShootSide.RIGHT;
    boolean commandsInitalized = false;

    @Override
    public void initialize() {
        super.reset();
    }

    private Pose getStartPose() {
        return new Pose();
    }

}
