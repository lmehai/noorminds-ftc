package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * NoorMinds Robotics - Gamepad Diagnostic Test
 *
 * Purpose:
 * Test all standard FTC gamepad inputs without moving the robot.
 *
 * Hardware:
 * - REV Driver Hub
 * - USB gamepad (PS2, PlayStation-style, or Xbox-style)
 *
 * Learning objectives:
 * 1. Understand how gamepad inputs work.
 * 2. Read joystick positions.
 * 3. Detect button presses.
 * 4. Display real-time telemetry.
 * 5. Troubleshoot gamepad connectivity.
 *
 * This program does not control motors or servos.
 */

@TeleOp(name = "NoorMindsGamepadTest", group = "NoorMinds")
public class NoorMindsGamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // STEP 1: Initialize telemetry.
        telemetry.addLine("=== NoorMinds Gamepad Test ===");
        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Instructions", "Press START on Driver Hub");
        telemetry.update();

        // STEP 2: Wait for the driver to start the test.
        waitForStart();

        // STEP 3: Continuously read all gamepad inputs.
        while (opModeIsActive()) {

            // Read joystick positions.
            float leftX = gamepad1.left_stick_x;
            float leftY = gamepad1.left_stick_y;

            float rightX = gamepad1.right_stick_x;
            float rightY = gamepad1.right_stick_y;

            // Detect whether a standard control is being operated.
            boolean inputActive =
                    Math.abs(leftX) > 0.05 ||
                            Math.abs(leftY) > 0.05 ||
                            Math.abs(rightX) > 0.05 ||
                            Math.abs(rightY) > 0.05 ||
                            gamepad1.a ||
                            gamepad1.b ||
                            gamepad1.x ||
                            gamepad1.y ||
                            gamepad1.dpad_up ||
                            gamepad1.dpad_down ||
                            gamepad1.dpad_left ||
                            gamepad1.dpad_right ||
                            gamepad1.left_bumper ||
                            gamepad1.right_bumper ||
                            gamepad1.left_trigger > 0.05 ||
                            gamepad1.right_trigger > 0.05 ||
                            gamepad1.left_stick_button ||
                            gamepad1.right_stick_button ||
                            gamepad1.start ||
                            gamepad1.back ||
                            gamepad1.guide;

            // STEP 4: Display gamepad identification.
            telemetry.addLine("=== NOORMINDS GAMEPAD TEST ===");

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Gamepad ID", gamepad1.getGamepadId());
            telemetry.addData("Input Active", inputActive);

            // STEP 5: Display joystick positions.
            telemetry.addLine("--- LEFT JOYSTICK ---");

            telemetry.addData("Left X", "%.3f", leftX);
            telemetry.addData("Left Y", "%.3f", leftY);
            telemetry.addData("Left Stick Press",
                    gamepad1.left_stick_button);

            telemetry.addLine("--- RIGHT JOYSTICK ---");

            telemetry.addData("Right X", "%.3f", rightX);
            telemetry.addData("Right Y", "%.3f", rightY);
            telemetry.addData("Right Stick Press",
                    gamepad1.right_stick_button);

            // STEP 6: Display the four action buttons.
            telemetry.addLine("--- ACTION BUTTONS ---");

            telemetry.addData("A / Cross", gamepad1.a);
            telemetry.addData("B / Circle", gamepad1.b);
            telemetry.addData("X / Square", gamepad1.x);
            telemetry.addData("Y / Triangle", gamepad1.y);

            // STEP 7: Display directional pad buttons.
            telemetry.addLine("--- D-PAD ---");

            telemetry.addData("D-Pad UP", gamepad1.dpad_up);
            telemetry.addData("D-Pad DOWN", gamepad1.dpad_down);
            telemetry.addData("D-Pad LEFT", gamepad1.dpad_left);
            telemetry.addData("D-Pad RIGHT", gamepad1.dpad_right);

            // STEP 8: Display shoulder buttons and triggers.
            telemetry.addLine("--- SHOULDER BUTTONS ---");

            telemetry.addData("L1 / Left Bumper",
                    gamepad1.left_bumper);

            telemetry.addData("R1 / Right Bumper",
                    gamepad1.right_bumper);

            telemetry.addData("L2 / Left Trigger", "%.3f",
                    gamepad1.left_trigger);

            telemetry.addData("R2 / Right Trigger", "%.3f",
                    gamepad1.right_trigger);

            // STEP 9: Display system buttons.
            telemetry.addLine("--- SYSTEM BUTTONS ---");

            telemetry.addData("START", gamepad1.start);
            telemetry.addData("SELECT / BACK", gamepad1.back);
            telemetry.addData("GUIDE / HOME", gamepad1.guide);

            // STEP 10: Update the Driver Hub display.
            telemetry.update();

            sleep(100);
        }
    }
}