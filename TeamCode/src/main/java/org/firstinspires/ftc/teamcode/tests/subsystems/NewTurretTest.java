package org.firstinspires.ftc.teamcode.tests.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Angle;

@TeleOp(name = "Turret Prototype Test", group = "Test")
@Config
@Configurable
public class NewTurretTest extends OpMode {

    Turret turret;

    public static double targetHeading = 0;


    @Override
    public void init() {
        turret = new Turret(hardwareMap, true);
    }

    @Override
    public void loop() {
        // Clamp heading range
        if (targetHeading > turret.getTotalRangeDegrees()) targetHeading = turret.getTotalRangeDegrees();
        if (targetHeading < 0) targetHeading = 0;

//        turret.angleToServo(Angle.fromDegrees(targetHeading));

        telemetry.addData("Target Heading", targetHeading);
        telemetry.update();
    }


}