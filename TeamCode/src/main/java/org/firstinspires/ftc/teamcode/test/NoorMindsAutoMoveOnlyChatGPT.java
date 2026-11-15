package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * NoorMinds Robotics
 *
 * Autonomous: Move Forward, Turn Left, Move Forward
 *
 * Robot: goBILDA FTC StarterBot
 *
 * Movement sequence:
 *   1. Forward 24 inches
 *   2. Turn left 90 degrees
 *   3. Forward 12 inches
 *   4. Stop
 *
 * Features:
 *   - Encoder-based movement
 *   - Detailed telemetry
 *   - Timeout detection
 *   - Encoder position diagnostics
 *   - Automatic shutdown on failure
 *   - Android RobotLog diagnostics
 *
 * IMPORTANT:
 * This implementation assumes TWO drive motors:
 * one controlling the left side and one controlling the right side.
 *
 * Verify motor names, directions, encoder specifications,
 * and drivetrain configuration before running.
 */
@Autonomous(
        name = "NoorMinds Auto - Move Only",
        group = "NoorMinds"
)
public class NoorMindsAutoMoveOnlyChatGPT extends LinearOpMode {

    // ============================================================
    // 1. LOGGING
    // ============================================================

    private static final String TAG = "NoorMindsAuto";

    // ============================================================
    // 2. HARDWARE CONFIGURATION
    // ============================================================

    // Must match the Control Hub hardware configuration.
    private static final String LEFT_MOTOR_NAME = "left_drive";
    private static final String RIGHT_MOTOR_NAME = "right_drive";

    // Verify these directions with the robot raised off the floor.
    private static final boolean LEFT_REVERSED = true;
    private static final boolean RIGHT_REVERSED = false;

    // ============================================================
    // 3. ENCODER AND WHEEL CONFIGURATION
    // ============================================================

    // Approximate encoder ticks per gearbox output revolution.
    // Verify the installed motor model.
    private static final double TICKS_PER_OUTPUT_REV = 537.7;

    // External drivetrain gearing.
    // 1.0 means one wheel revolution per gearbox output revolution.
    private static final double EXTERNAL_GEAR_RATIO = 1.0;

    // Wheel diameter in millimeters.
    private static final double WHEEL_DIAMETER_MM = 96.0;

    // Physical distance between left and right wheel centerlines.
    // Measure and calibrate for your robot.
    private static final double TRACK_WIDTH_INCHES = 12.0;

    // Calibration factors.
    private static final double DISTANCE_CALIBRATION = 1.0;
    private static final double TURN_CALIBRATION = 1.0;

    private static final double WHEEL_DIAMETER_INCHES =
            WHEEL_DIAMETER_MM / 25.4;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_OUTPUT_REV
                    * EXTERNAL_GEAR_RATIO
                    * DISTANCE_CALIBRATION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ============================================================
    // 4. AUTONOMOUS MOVEMENT CONFIGURATION
    // ============================================================

    private static final double FIRST_FORWARD_INCHES = 24.0;
    private static final double TURN_LEFT_DEGREES = 90.0;
    private static final double SECOND_FORWARD_INCHES = 12.0;

    private static final double DRIVE_SPEED = 0.45;
    private static final double TURN_SPEED = 0.35;

    private static final double DRIVE_TIMEOUT_SECONDS = 5.0;
    private static final double TURN_TIMEOUT_SECONDS = 4.0;

    // Pause between successful movements.
    private static final long MOVEMENT_PAUSE_MS = 250;

    // Telemetry refresh interval.
    private static final long TELEMETRY_INTERVAL_MS = 100;

    // Maximum allowed encoder error after a movement.
    // This is a diagnostic threshold, not a physical distance guarantee.
    private static final int POSITION_TOLERANCE_TICKS = 15;

    // ============================================================
    // 5. HARDWARE OBJECTS
    // ============================================================

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // ============================================================
    // 6. TIMERS
    // ============================================================

    private final ElapsedTime movementTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime autonomousTimer = new ElapsedTime();

    // ============================================================
    // 7. AUTONOMOUS ENTRY POINT
    // ============================================================

    @Override
    public void runOpMode() {

        try {

            initializeHardware();

            displayInitializationTelemetry();

            telemetry.addLine("Waiting for START...");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            autonomousTimer.reset();

            logInfo("Autonomous STARTED");

            // ----------------------------------------------------
            // STEP 1: Forward 24 inches
            // ----------------------------------------------------

            if (!driveForward(
                    FIRST_FORWARD_INCHES,
                    "STEP 1 - Forward 24 inches")) {

                reportFailure("STEP 1 FAILED");
                return;
            }

            sleep(MOVEMENT_PAUSE_MS);

            if (!opModeIsActive()) {
                return;
            }

            // ----------------------------------------------------
            // STEP 2: Turn left 90 degrees
            // ----------------------------------------------------

            if (!turnLeft(
                    TURN_LEFT_DEGREES,
                    "STEP 2 - Turn left 90 degrees")) {

                reportFailure("STEP 2 FAILED");
                return;
            }

            sleep(MOVEMENT_PAUSE_MS);

            if (!opModeIsActive()) {
                return;
            }

            // ----------------------------------------------------
            // STEP 3: Forward 12 inches
            // ----------------------------------------------------

            if (!driveForward(
                    SECOND_FORWARD_INCHES,
                    "STEP 3 - Forward 12 inches")) {

                reportFailure("STEP 3 FAILED");
                return;
            }

            // ----------------------------------------------------
            // AUTONOMOUS COMPLETE
            // ----------------------------------------------------

            stopMotors();

            telemetry.clearAll();
            telemetry.addLine("AUTONOMOUS COMPLETE");
            telemetry.addData(
                    "Total Time",
                    "%.2f seconds",
                    autonomousTimer.seconds()
            );
            telemetry.addData(
                    "Final Left Encoder",
                    leftMotor.getCurrentPosition()
            );
            telemetry.addData(
                    "Final Right Encoder",
                    rightMotor.getCurrentPosition()
            );
            telemetry.update();

            logInfo(
                    String.format("Autonomous COMPLETE in %.2f seconds",
                            autonomousTimer.seconds()
                    )
            );

            sleep(1000);

        } catch (Exception e) {

            RobotLog.ee(TAG, e, "Autonomous exception");

            telemetry.clearAll();
            telemetry.addLine("AUTONOMOUS ERROR");
            telemetry.addData("Exception", e.getClass().getSimpleName());
            telemetry.addData("Message", e.getMessage());
            telemetry.update();

        } finally {

            stopMotors();
            logInfo("Autonomous stopped");
        }
    }

    // ============================================================
    // 8. HARDWARE INITIALIZATION
    // ============================================================

    private void initializeHardware() {

        logInfo("Initializing hardware");

        leftMotor = hardwareMap.get(
                DcMotor.class,
                LEFT_MOTOR_NAME
        );

        rightMotor = hardwareMap.get(
                DcMotor.class,
                RIGHT_MOTOR_NAME
        );

        leftMotor.setDirection(
                LEFT_REVERSED
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD
        );

        rightMotor.setDirection(
                RIGHT_REVERSED
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD
        );

        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // Ensure motors are stopped before resetting encoders.
        stopMotors();

        leftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        leftMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        logInfo("Hardware initialized successfully");
    }

    // ============================================================
    // 9. FORWARD MOVEMENT
    // ============================================================

    private boolean driveForward(
            double inches,
            String stepName) {

        return encoderDrive(
                DRIVE_SPEED,
                inches,
                inches,
                DRIVE_TIMEOUT_SECONDS,
                stepName
        );
    }

    // ============================================================
    // 10. LEFT TURN
    // ============================================================

    private boolean turnLeft(
            double degrees,
            String stepName) {

        double wheelDistance =
                Math.PI
                        * TRACK_WIDTH_INCHES
                        * (degrees / 360.0)
                        * TURN_CALIBRATION;

        return encoderDrive(
                TURN_SPEED,
                -wheelDistance,
                wheelDistance,
                TURN_TIMEOUT_SECONDS,
                stepName
        );
    }

    // ============================================================
    // 11. ENCODER MOVEMENT ENGINE
    // ============================================================

    private boolean encoderDrive(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutSeconds,
            String stepName) {

        if (!opModeIsActive()) {
            return false;
        }

        // --------------------------------------------------------
        // Calculate target encoder positions.
        // --------------------------------------------------------

        int leftStart = leftMotor.getCurrentPosition();
        int rightStart = rightMotor.getCurrentPosition();

        int leftTarget =
                leftStart
                        + (int) Math.round(
                        leftInches * TICKS_PER_INCH
                );

        int rightTarget =
                rightStart
                        + (int) Math.round(
                        rightInches * TICKS_PER_INCH
                );

        logInfo(
                String.format(
                        "%s START | Left: %d -> %d | Right: %d -> %d",
                        stepName,
                        leftStart,
                        leftTarget,
                        rightStart,
                        rightTarget
                )
        );

        // --------------------------------------------------------
        // Configure target positions.
        // --------------------------------------------------------

        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        // --------------------------------------------------------
        // Start motors.
        // --------------------------------------------------------

        movementTimer.reset();
        telemetryTimer.reset();

        double motorPower =
                Math.max(0.0, Math.min(1.0, Math.abs(speed)));

        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);

        boolean timedOut = false;
        boolean interrupted = false;

        try {

            // ----------------------------------------------------
            // Monitor movement.
            // ----------------------------------------------------

            while (opModeIsActive()
                    && movementTimer.seconds() < timeoutSeconds
                    && (leftMotor.isBusy() || rightMotor.isBusy())) {

                if (telemetryTimer.milliseconds()
                        >= TELEMETRY_INTERVAL_MS) {

                    displayMovementTelemetry(
                            stepName,
                            leftStart,
                            rightStart,
                            leftTarget,
                            rightTarget,
                            timeoutSeconds
                    );

                    telemetryTimer.reset();
                }

                idle();
            }

            // ----------------------------------------------------
            // Determine why movement ended.
            // ----------------------------------------------------

            interrupted = !opModeIsActive();

            timedOut =
                    !interrupted
                            && movementTimer.seconds() >= timeoutSeconds
                            && (leftMotor.isBusy() || rightMotor.isBusy());

        } finally {

            // Always stop both motors.
            stopMotors();

            // Restore encoder operating mode.
            leftMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );

            rightMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );
        }

        // --------------------------------------------------------
        // Calculate final encoder errors.
        // --------------------------------------------------------

        int leftFinal = leftMotor.getCurrentPosition();
        int rightFinal = rightMotor.getCurrentPosition();

        int leftError =
                Math.abs(leftTarget - leftFinal);

        int rightError =
                Math.abs(rightTarget - rightFinal);

        boolean withinTolerance =
                leftError <= POSITION_TOLERANCE_TICKS
                        && rightError <= POSITION_TOLERANCE_TICKS;

        boolean success =
                !interrupted
                        && !timedOut
                        && withinTolerance;

        // --------------------------------------------------------
        // Display final movement result.
        // --------------------------------------------------------

        displayMovementResult(
                stepName,
                leftTarget,
                rightTarget,
                leftFinal,
                rightFinal,
                leftError,
                rightError,
                timedOut,
                interrupted,
                success
        );

        return success;
    }

    // ============================================================
    // 12. INITIALIZATION TELEMETRY
    // ============================================================

    private void displayInitializationTelemetry() {

        telemetry.clearAll();

        telemetry.addLine("NOORMINDS ROBOTICS");
        telemetry.addLine("AUTONOMOUS INITIALIZATION");
        telemetry.addLine("-----------------------------");

        telemetry.addData("Status", "READY");

        telemetry.addData(
                "Left Motor",
                LEFT_MOTOR_NAME
        );

        telemetry.addData(
                "Right Motor",
                RIGHT_MOTOR_NAME
        );

        telemetry.addData(
                "Left Direction",
                leftMotor.getDirection()
        );

        telemetry.addData(
                "Right Direction",
                rightMotor.getDirection()
        );

        telemetry.addData(
                "Ticks Per Inch",
                "%.2f",
                TICKS_PER_INCH
        );

        telemetry.addData(
                "Left Encoder",
                leftMotor.getCurrentPosition()
        );

        telemetry.addData(
                "Right Encoder",
                rightMotor.getCurrentPosition()
        );

        telemetry.addLine("-----------------------------");
        telemetry.addLine("Press PLAY to start");

        telemetry.update();
    }

    // ============================================================
    // 13. LIVE MOVEMENT TELEMETRY
    // ============================================================

    private void displayMovementTelemetry(
            String stepName,
            int leftStart,
            int rightStart,
            int leftTarget,
            int rightTarget,
            double timeoutSeconds) {

        int leftCurrent = leftMotor.getCurrentPosition();
        int rightCurrent = rightMotor.getCurrentPosition();

        int leftError = leftTarget - leftCurrent;
        int rightError = rightTarget - rightCurrent;

        int leftRequested = leftTarget - leftStart;
        int rightRequested = rightTarget - rightStart;

        int leftTravelled = leftCurrent - leftStart;
        int rightTravelled = rightCurrent - rightStart;

        telemetry.clearAll();

        telemetry.addLine("NOORMINDS AUTONOMOUS");
        telemetry.addLine("-----------------------------");

        telemetry.addData("Step", stepName);
        telemetry.addData("Status", "MOVING");

        telemetry.addData(
                "Time",
                "%.2f / %.2f sec",
                movementTimer.seconds(),
                timeoutSeconds
        );

        telemetry.addLine("-----------------------------");
        telemetry.addLine("LEFT MOTOR");

        telemetry.addData("Start", leftStart);
        telemetry.addData("Target", leftTarget);
        telemetry.addData("Current", leftCurrent);
        telemetry.addData("Error", leftError);
        telemetry.addData("Busy", leftMotor.isBusy());

        telemetry.addData(
                "Travel",
                "%d / %d ticks",
                leftTravelled,
                leftRequested
        );

        telemetry.addLine("-----------------------------");
        telemetry.addLine("RIGHT MOTOR");

        telemetry.addData("Start", rightStart);
        telemetry.addData("Target", rightTarget);
        telemetry.addData("Current", rightCurrent);
        telemetry.addData("Error", rightError);
        telemetry.addData("Busy", rightMotor.isBusy());

        telemetry.addData(
                "Travel",
                "%d / %d ticks",
                rightTravelled,
                rightRequested
        );

        telemetry.addLine("-----------------------------");

        telemetry.addData(
                "Motor Power",
                "%.2f / %.2f",
                leftMotor.getPower(),
                rightMotor.getPower()
        );

        telemetry.update();
    }

    // ============================================================
    // 14. MOVEMENT RESULT TELEMETRY
    // ============================================================

    private void displayMovementResult(
            String stepName,
            int leftTarget,
            int rightTarget,
            int leftFinal,
            int rightFinal,
            int leftError,
            int rightError,
            boolean timedOut,
            boolean interrupted,
            boolean success) {

        String result;

        if (interrupted) {
            result = "INTERRUPTED";
        } else if (timedOut) {
            result = "TIMEOUT";
        } else if (!success) {
            result = "POSITION ERROR";
        } else {
            result = "SUCCESS";
        }

        telemetry.clearAll();

        telemetry.addLine("MOVEMENT RESULT");
        telemetry.addLine("-----------------------------");

        telemetry.addData("Step", stepName);
        telemetry.addData("Result", result);

        telemetry.addData(
                "Duration",
                "%.2f sec",
                movementTimer.seconds()
        );

        telemetry.addLine("-----------------------------");

        telemetry.addData("Left Target", leftTarget);
        telemetry.addData("Left Final", leftFinal);
        telemetry.addData("Left Error", leftError);

        telemetry.addLine("-----------------------------");

        telemetry.addData("Right Target", rightTarget);
        telemetry.addData("Right Final", rightFinal);
        telemetry.addData("Right Error", rightError);

        telemetry.addLine("-----------------------------");

        telemetry.addData(
                "Tolerance",
                "%d ticks",
                POSITION_TOLERANCE_TICKS
        );

        telemetry.update();

        logInfo(
                String.format(
                        "%s | Result=%s | Left=%d/%d Error=%d | "
                                + "Right=%d/%d Error=%d | Time=%.2f",
                        stepName,
                        result,
                        leftFinal,
                        leftTarget,
                        leftError,
                        rightFinal,
                        rightTarget,
                        rightError,
                        movementTimer.seconds()
                )
        );
    }

    // ============================================================
    // 15. FAILURE REPORTING
    // ============================================================

    private void reportFailure(String message) {

        stopMotors();

        telemetry.addLine("-----------------------------");
        telemetry.addData("AUTONOMOUS", "ABORTED");
        telemetry.addData("Reason", message);
        telemetry.update();

        RobotLog.ee(TAG, message);
    }

    // ============================================================
    // 16. MOTOR SHUTDOWN
    // ============================================================

    private void stopMotors() {

        if (leftMotor != null) {
            leftMotor.setPower(0);
        }

        if (rightMotor != null) {
            rightMotor.setPower(0);
        }
    }

    // ============================================================
    // 17. LOGGING
    // ============================================================

    private void logInfo(String message) {

        RobotLog.ii(TAG, message);
    }
}