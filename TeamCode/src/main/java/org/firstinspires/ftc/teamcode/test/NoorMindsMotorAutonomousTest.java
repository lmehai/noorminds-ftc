package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * NoorMinds Robotics - Motor Test
 *
 * Motor: goBILDA 5203 Series
 * Controller: REV Control Hub
 * Motor Port: 0
 *
 * Objective:
 * 1. Rotate the motor forward for 2 seconds.
 * 2. Stop for 1 second.
 * 3. Rotate the motor backward for 2 seconds.
 * 4. Stop the motor.
 */

@Autonomous(name = "NoorMindsMotorAutonomousTest", group = "NoorMinds")
public class NoorMindsMotorAutonomousTest extends LinearOpMode {

    // Motor connected to the REV Control Hub.
    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        // STEP 1: Initialize the motor using the robot configuration.
        motor = hardwareMap.get(DcMotor.class, "testMotor");

        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.0);

        // STEP 2: Inform the driver that the robot is ready.
        telemetry.addData("Motor", "goBILDA 5203");
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        try {
            // STEP 3: Rotate the motor forward at 20% power.
            if (opModeIsActive()) {
                telemetry.addData("Status", "Moving Forward");
                telemetry.update();

                motor.setPower(0.20);
                sleep(2000);
            }

            // STEP 4: Stop the motor for 1 second.
            if (opModeIsActive()) {
                motor.setPower(0.0);
                sleep(1000);
            }

            // STEP 5: Rotate the motor backward at 20% power.
            if (opModeIsActive()) {
                telemetry.addData("Status", "Moving Backward");
                telemetry.update();

                motor.setPower(-0.20);
                sleep(2000);
            }

        } finally {
            // STEP 6: Always stop the motor when the test ends.
            motor.setPower(0.0);
        }

        telemetry.addData("Status", "Test Completed");
        telemetry.update();
    }
}