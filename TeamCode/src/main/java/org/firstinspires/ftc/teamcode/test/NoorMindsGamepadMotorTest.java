package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * NoorMinds Robotics - Gamepad Motor Test
 *
 * Purpose:
 * Control one DC motor using Gamepad 1 and display
 * diagnostic information on the Driver Hub.
 *
 * Hardware:
 * - REV Control Hub, Motor Port 0
 * - Motor configuration name: testMotor
 * - Gamepad connected to the Driver Hub
 *
 * Controls:
 * - Left joystick UP/DOWN: Motor direction and power
 * - Button A: Slow mode (30%)
 * - Button B: Normal mode (70%)
 * - Button X: Disable motor
 * - Button A or B: Re-enable motor
 *
 * Learning objectives:
 * - Read gamepad input
 * - Control motor power
 * - Use variables, conditions, and loops
 * - Debug using telemetry
 */

@TeleOp(name = "NoorMindsGamepadMotorTest", group = "NoorMinds")
public class NoorMindsGamepadMotorTest extends LinearOpMode {

    // Motor hardware and control constants.
    private DcMotor motor;

    private static final double SLOW_SPEED = 0.30;
    private static final double NORMAL_SPEED = 0.70;
    private static final double DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {

        // STEP 1: Initialize the motor from the robot configuration.
        motor = hardwareMap.get(DcMotor.class, "testMotor");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);

        // STEP 2: Initialize motor control variables.
        double maxSpeed = SLOW_SPEED;
        boolean motorEnabled = true;

        telemetry.addData("Robot", "NoorMinds Robotics");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Motor", "testMotor");
        telemetry.addData("Gamepad", "Use Gamepad 1");
        telemetry.addData("Controls", "A=Slow B=Normal X=Disable");
        telemetry.addData("Next Step", "Press START on Driver Hub");
        telemetry.update();

        // STEP 3: Wait for the driver to start the program.
        waitForStart();

        try {

            // STEP 4: Continuously process gamepad input.
            while (opModeIsActive()) {

                // Read joystick position.
                // FTC returns negative Y when pushed upward.
                double rawJoystickY = gamepad1.left_stick_y;
                double joystick = -rawJoystickY;

                // STEP 5: Read buttons and select the speed mode.
                if (gamepad1.a) {
                    maxSpeed = SLOW_SPEED;
                    motorEnabled = true;
                }

                if (gamepad1.b) {
                    maxSpeed = NORMAL_SPEED;
                    motorEnabled = true;
                }

                if (gamepad1.x) {
                    motorEnabled = false;
                }

                // STEP 6: Ignore small joystick movements.
                if (Math.abs(joystick) < DEADZONE) {
                    joystick = 0.0;
                }

                // Calculate the requested motor power.
                double motorPower = joystick * maxSpeed;

                if (!motorEnabled) {
                    motorPower = 0.0;
                }

                // STEP 7: Send the power command to the motor.
                motor.setPower(motorPower);

                // STEP 8: Display diagnostic telemetry.
                telemetry.addLine("=== NoorMinds Motor Test ===");

                telemetry.addData("Status", "RUNNING");
                telemetry.addData("Motor Name", "testMotor");

                telemetry.addLine("--- Gamepad Input ---");
                telemetry.addData("Gamepad ID", gamepad1.getGamepadId());
                telemetry.addData("Left Joystick Y", "%.3f", rawJoystickY);
                telemetry.addData("Button A", gamepad1.a);
                telemetry.addData("Button B", gamepad1.b);
                telemetry.addData("Button X", gamepad1.x);

                telemetry.addLine("--- Motor Control ---");
                telemetry.addData("Motor Enabled", motorEnabled);
                telemetry.addData("Max Speed", "%.0f%%", maxSpeed * 100);
                telemetry.addData("Motor Power", "%.3f", motorPower);
                telemetry.addData("Motor Reported Power",
                        "%.3f", motor.getPower());

                telemetry.addLine("--- Instructions ---");
                telemetry.addData("Joystick", "UP=Forward DOWN=Reverse");
                telemetry.addData("Buttons", "A=30%% B=70%% X=Disable");

                telemetry.update();

                // Allow the FTC runtime to perform other tasks.
                idle();
            }

        } finally {

            // STEP 9: Stop the motor when the program ends.
            motor.setPower(0.0);
        }
    }
}