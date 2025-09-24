package org.firstinspires.ftc.teamcode.OutdatedTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class GamepadTesting extends OpMode{

    // This is used for later in the lesson
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        //There are 2 gamepads. This is because most robots are controlled by 2 drivers for efficiency
        // We will only be using gamepad1 for this demonstration
        //Everything here is pretty self explanatory. I'll try to explain anything that looks confusing
        //Honestly this would make more sense if there was a photo or something

        //Left Stick
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Left stick button", gamepad1.left_stick_button); // When you push the left joystick (you feel a click when pushing it)

        //Right Stick
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("Right stick button", gamepad1.right_stick_button); // When you push the right joystick (you feel a click when pushing it)

        //Left Trigger
        telemetry.addData("Left trigger", gamepad1.left_trigger);
        telemetry.addData("Left bumper", gamepad1.left_bumper);

        //Right Trigger
        telemetry.addData("Right trigger", gamepad1.right_trigger);
        telemetry.addData("Right bumper", gamepad1.right_bumper);

        // Buttons (There is also the shapes for playstation controller but im too lazy to add it)
        telemetry.addData("Y button", gamepad1.y);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("X button", gamepad1.x);
        telemetry.addData("A button", gamepad1.a);

        // Dpad (the plus thingy on the controller)
        telemetry.addData("Dpad up", gamepad1.dpad_up);
        telemetry.addData("Dpad down", gamepad1.dpad_down);
        telemetry.addData("Dpad left", gamepad1.dpad_left);
        telemetry.addData("Dpad right", gamepad1.dpad_right);

        //Touch pad (pretty cool but kinda useless ngl)
        //There are 2 fingers for when more than one finger is on the touchpad
        telemetry.addData("Finger 1 Touchpad x",gamepad1.touchpad_finger_1_x);
        telemetry.addData("Finger 1 Touchpad y",gamepad1.touchpad_finger_1_y);
        telemetry.addData("Finger 2 Touchpad x",gamepad1.touchpad_finger_2_x);
        telemetry.addData("Finger 2 Touchpad y",gamepad1.touchpad_finger_2_y);

        //Misc
        telemetry.addData("Start",gamepad1.start); // For playstation its the options button
        telemetry.addData("Back",gamepad1.back); // For playstation its the share button

        // Used to check if button has already been pressed
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.a && !previousGamepad1.a) { //This is known as a rising edge detector. Its simply checks if the button is pressed and only runs once
            gamepad1.rumble(1000); //vibrates the gamepad for 1 second
        }

        if (!currentGamepad1.b && previousGamepad1.b) { //This is known as a falling edge detector. Same thing as rising, but only when button is released
            gamepad1.rumble(1000); //vibrates the gamepad for 1 second
        }

        // I spent way too much time on this lesson
    }
}
