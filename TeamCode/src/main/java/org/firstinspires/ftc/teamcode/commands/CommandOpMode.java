package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;

/**
 * As opposed to the general WPILib-style Robot paradigm, SolversLib also offers a command opmode
 * for individual opmodes.
 *
 * @author Jackson
 */
public abstract class CommandOpMode extends LinearOpMode {

    /**
     * Cancels all previous commands
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs the {@link CommandScheduler} instance
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules {@link Command} objects to the scheduler
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Registers {@link Subsystem} objects to the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // run the scheduler
        try {
            while (opModeInInit()) {
                initialize_loop();
            }
            first();
            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            try {
                end();
            } finally {
                reset();
            }
        }
    }

    public abstract void initialize();

    public void first() { };

    /**
     * Runs at the end (when opMode is no longer active) of CommandOpMode
     */
    public void end() { }

    public void initialize_loop() { }

    public static void disable() {
        Robot.disable();
    }

    public static void enable() {
        Robot.enable();
    }
}
