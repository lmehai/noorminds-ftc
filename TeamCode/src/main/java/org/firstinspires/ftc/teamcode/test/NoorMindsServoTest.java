package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * ============================================================
 * NOORMINDS ROBOTICS
 * Generic FTC Servo Test
 * ============================================================
 *
 * PURPOSE:
 * Learn how to control different types of servos using Java.
 *
 * SUPPORTED HARDWARE:
 * - REV Smart Robot Servo (SRS)
 * - goBILDA servos
 * - Other FTC-compatible servos
 *
 * This program uses only standard FTC SDK interfaces.
 *
 * TWO SERVO MODES:
 *
 * 1. POSITIONAL
 *    Move a servo to a selected position.
 *
 *    Position range: 0.0 to 1.0
 *
 * 2. CONTINUOUS ROTATION
 *    Rotate a servo forward or backward.
 *
 *    Power range: -1.0 to 1.0
 *
 * IMPORTANT:
 * The servo must already be configured for the selected
 * operating mode.
 *
 * GAMEPAD CONTROLS:
 *
 * X (Cross) = Forward / Increase position
 * Circle    = Reverse / Decrease position
 * Square    = Stop (continuous mode only)
 *
 * D-Pad Up   = Increase power or step size
 * D-Pad Down = Decrease power or step size
 *
 * ============================================================
 */

@TeleOp(
        name = "NoorMinds - Generic Servo Test",
        group = "NoorMinds"
)
public class NoorMindsServoTest extends OpMode {

    // ========================================================
    // 1. STUDENT CONFIGURATION
    // ========================================================

    /*
     * Change these values to match your hardware.
     *
     * Examples:
     *
     * "testServo"
     * "windmill"
     * "left_intake_servo"
     */

    private static final String SERVO_NAME = "testServo";

    /*
     * true  = Continuous rotation servo
     * false = Positional servo
     *
     * For REV SRS:
     * Select the mode matching its physical configuration.
     */

    private static final boolean CONTINUOUS_ROTATION = false;

    // Starting power for continuous rotation mode.
    private static final double DEFAULT_POWER = 0.30;

    // Starting position for positional mode.
    private static final double DEFAULT_POSITION = 0.50;

    // Adjustment increment.
    private static final double DEFAULT_STEP = 0.10;

    // ========================================================
    // 2. SERVO OBJECTS
    // ========================================================

    /*
     * Servo:
     * Used for positional servos.
     *
     * CRServo:
     * Used for continuous rotation servos.
     *
     * We only initialize the one needed.
     */

    private Servo positionalServo;
    private CRServo continuousServo;

    // ========================================================
    // 3. VARIABLES
    // ========================================================

    private double power = DEFAULT_POWER;
    private double position = DEFAULT_POSITION;
    private double step = DEFAULT_STEP;

    // Actual command sent to a continuous servo.
    private double commandedPower = 0.0;

    // Text describing the current action.
    private String status = "READY";

    // Remember previous button states.
    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    // ========================================================
    // 4. INIT
    // ========================================================

    /**
     * Called ONCE when INIT is pressed.
     *
     * Connect the Java program to the physical servo.
     */

    @Override
    public void init() {

        if (CONTINUOUS_ROTATION) {

            // Get continuous rotation servo.
            continuousServo = hardwareMap.get(
                    CRServo.class,
                    SERVO_NAME
            );

            // Make sure servo is stopped.
            continuousServo.setPower(0.0);

        } else {

            // Get positional servo.
            positionalServo = hardwareMap.get(
                    Servo.class,
                    SERVO_NAME
            );

            /*
             * Do not move the positional servo during INIT.
             *
             * Its first commanded position will be sent
             * when the student presses a control button.
             */
        }

        telemetry.addLine("NOORMINDS GENERIC SERVO TEST");

        telemetry.addData("Status", "INITIALIZED");
        telemetry.addData("Servo Name", SERVO_NAME);

        telemetry.addData(
                "Servo Mode",
                CONTINUOUS_ROTATION
                        ? "CONTINUOUS"
                        : "POSITIONAL"
        );

        telemetry.addLine("Press START to begin");
        telemetry.update();
    }

    // ========================================================
    // 5. LOOP
    // ========================================================

    /**
     * Called REPEATEDLY after START.
     *
     * The loop:
     *
     * 1. Reads gamepad buttons.
     * 2. Adjusts settings.
     * 3. Controls the servo.
     * 4. Displays telemetry.
     */

    @Override
    public void loop() {

        // ----------------------------------------------------
        // STEP A: Adjust power or step size
        // ----------------------------------------------------

        /*
         * Detect a NEW button press.
         *
         * Without this check, the value could change
         * many times while a button is held.
         */

        if (gamepad1.dpad_up && !previousUp) {

            if (CONTINUOUS_ROTATION) {

                power = Math.min(
                        1.0,
                        power + 0.10
                );

            } else {

                step = Math.min(
                        0.50,
                        step + 0.05
                );
            }
        }

        if (gamepad1.dpad_down && !previousDown) {

            if (CONTINUOUS_ROTATION) {

                power = Math.max(
                        0.0,
                        power - 0.10
                );

            } else {

                step = Math.max(
                        0.05,
                        step - 0.05
                );
            }
        }

        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;

        // ----------------------------------------------------
        // STEP B: Control servo
        // ----------------------------------------------------

        if (CONTINUOUS_ROTATION) {

            controlContinuousServo();

        } else {

            controlPositionalServo();
        }

        // ----------------------------------------------------
        // STEP C: Display telemetry
        // ----------------------------------------------------

        displayTelemetry();
    }

    // ========================================================
    // 6. CONTINUOUS ROTATION CONTROL
    // ========================================================

    /**
     * Controls a continuous rotation servo.
     *
     * X (Cross) = Forward
     * Circle    = Reverse
     * Square    = Stop
     *
     * Releasing the direction button also stops the servo.
     */

    private void controlContinuousServo() {

        if (gamepad1.x) {

            // Square button.
            commandedPower = 0.0;
            status = "STOPPED";

        } else if (gamepad1.a && !gamepad1.b) {

            // X (Cross) button.
            commandedPower = power;
            status = "FORWARD";

        } else if (gamepad1.b && !gamepad1.a) {

            // Circle button.
            commandedPower = -power;
            status = "REVERSE";

        } else {

            // No direction button pressed.
            commandedPower = 0.0;
            status = "STOPPED";
        }

        continuousServo.setPower(commandedPower);
    }

    // ========================================================
    // 7. POSITIONAL SERVO CONTROL
    // ========================================================

    /**
     * Controls a positional servo.
     *
     * X (Cross) = Increase position
     * Circle    = Decrease position
     *
     * Position must remain between 0.0 and 1.0.
     *
     * Unlike a continuous servo, a positional servo
     * holds its commanded position when buttons are released.
     */

    private void controlPositionalServo() {

        // Increase position once per button press.
        if (gamepad1.a && !previousA) {

            position = Math.min(
                    1.0,
                    position + step
            );

            positionalServo.setPosition(position);

            status = "POSITION INCREASED";
        }

        // Decrease position once per button press.
        if (gamepad1.b && !previousB) {

            position = Math.max(
                    0.0,
                    position - step
            );

            positionalServo.setPosition(position);

            status = "POSITION DECREASED";
        }

        previousA = gamepad1.a;
        previousB = gamepad1.b;
    }

    // ========================================================
    // 8. TELEMETRY
    // ========================================================

    /**
     * Display information on the Driver Station.
     *
     * Telemetry helps students understand what their
     * program is doing.
     */

    private void displayTelemetry() {

        telemetry.addLine("NOORMINDS SERVO TEST");
        telemetry.addLine("========================");

        telemetry.addData("Servo", SERVO_NAME);

        telemetry.addData(
                "Mode",
                CONTINUOUS_ROTATION
                        ? "CONTINUOUS"
                        : "POSITIONAL"
        );

        telemetry.addData("Status", status);

        telemetry.addLine("------------------------");

        if (CONTINUOUS_ROTATION) {

            telemetry.addData(
                    "Selected Power",
                    "%.2f",
                    power
            );

            telemetry.addData(
                    "Commanded Power",
                    "%.2f",
                    commandedPower
            );

            telemetry.addLine("X (Cross): Forward");
            telemetry.addLine("Circle: Reverse");
            telemetry.addLine("Square: Stop");

        } else {

            telemetry.addData(
                    "Selected Position",
                    "%.2f",
                    position
            );

            telemetry.addData(
                    "Position Step",
                    "%.2f",
                    step
            );

            telemetry.addLine("X (Cross): Increase position");
            telemetry.addLine("Circle: Decrease position");
        }

        telemetry.addLine("------------------------");

        telemetry.addData("Gamepad A", gamepad1.a);
        telemetry.addData("Gamepad B", gamepad1.b);
        telemetry.addData("Gamepad X", gamepad1.x);

        telemetry.addData(
                "D-Pad Up",
                gamepad1.dpad_up
        );

        telemetry.addData(
                "D-Pad Down",
                gamepad1.dpad_down
        );

        telemetry.update();
    }

    // ========================================================
    // 9. STOP
    // ========================================================

    /**
     * Called when STOP is pressed.
     *
     * Continuous servos must be stopped.
     *
     * Positional servos retain their last commanded
     * position. We do not automatically move them.
     */

    @Override
    public void stop() {

        if (CONTINUOUS_ROTATION
                && continuousServo != null) {

            continuousServo.setPower(0.0);
        }
    }
}