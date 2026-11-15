package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * ============================================================================
 *  NoorMinds - goBILDA FTC StarterBot - "Move Only" autonomous program
 * ============================================================================
 *
 *  WHAT DOES THIS PROGRAM DO?
 *  The robot drives by itself (no controller!) like this:
 *
 *      1. Drive FORWARD 2 feet (24 inches)
 *      2. Turn LEFT (a quarter turn, 90 degrees)
 *      3. Drive FORWARD 1 foot (12 inches)
 *      4. STOP
 *
 *  WHAT IS AN "AUTONOMOUS" PROGRAM?
 *  "Autonomous" means the robot follows the program all by itself. In an FTC
 *  match the robot runs on its own for the first 30 seconds.
 *
 *  HOW DOES THE ROBOT KNOW HOW FAR IT WENT?
 *  Each drive motor has an ENCODER. An encoder is a tiny counter inside the
 *  motor. It counts "ticks" as the motor shaft spins - like counting how many
 *  times a wheel went around. If we know how many ticks make 1 inch, we can
 *  tell the motor: "spin until you have counted this many ticks" and it stops
 *  at exactly the right distance.
 *
 *  HOW DOES THE ROBOT TURN?
 *  Our robot has wheels on the left side and the right side (this is called
 *  "tank drive"). To turn LEFT in place, we spin the left wheels BACKWARD and
 *  the right wheels FORWARD at the same time.
 *
 *  BEFORE YOU PRESS PLAY (checklist)
 *  [ ] 1. Put the robot on blocks or hold it up the first time. Wheels will spin!
 *  [ ] 2. Make sure the motor names in the Driver Hub configuration match
 *         LEFT_DRIVE_NAME and RIGHT_DRIVE_NAME below (spelling counts!).
 *  [ ] 3. Test: do BOTH sides drive FORWARD? If a side goes backward, change
 *         true/false for that side in LEFT_REVERSED / RIGHT_REVERSED.
 *  [ ] 4. Drive 2 feet. Measure with a tape measure. Too short or too long?
 *         Adjust GEAR_REDUCTION or WHEEL_DIAMETER_MM (see notes below).
 *  [ ] 5. Is the left turn not a quarter turn? Adjust TRACK_WIDTH_INCHES.
 *
 *  This file follows the same pattern as the official FIRST sample
 *  "RobotAutoDriveByEncoder_Linear" (a "LinearOpMode", RUN_TO_POSITION, and an
 *  encoderDrive method), so you can compare the two to learn more.
 */
@Autonomous(name = "NoorMinds Auto - Move Only", group = "NoorMinds")
public class NoorMindsAutoMoveOnlyClaude extends LinearOpMode {

    // ========================================================================
    //  SETTINGS YOU MIGHT NEED TO CHANGE  (all in one place on purpose!)
    // ========================================================================

    // ---- 1. Motor names ----------------------------------------------------
    // These must be spelled EXACTLY like the names you typed in the Driver Hub
    // configuration (Configure Robot -> your Control Hub -> Motors).
    private static final String LEFT_DRIVE_NAME  = "left_drive";
    private static final String RIGHT_DRIVE_NAME = "right_drive";

    // ---- 2. Motor directions -----------------------------------------------
    // The left and right motors face opposite ways, so one of them has to spin
    // "backwards" to make the robot go forward. If the robot drives backward or
    // spins in a circle when it should go straight, swap true/false.
    private static final boolean LEFT_MOTOR_REVERSED  = true;
    private static final boolean RIGHT_MOTOR_REVERSED = false;

    // ---- 3. Math for turning "inches" into "encoder ticks" ------------------
    // Ticks the encoder counts for ONE full spin of the motor shaft.
    // goBILDA 19.2:1 Yellow Jacket motor = 537.7 ticks per spin.
    private static final double TICKS_PER_MOTOR_REV = 537.7;

    // Extra gears/chain between the motor and the wheel.
    // The StarterBot manual uses 30-tooth gear -> 30-tooth gear (1:1) and
    // 10-tooth sprocket -> 10-tooth sprocket (1:1), so 1.0 is a good start.
    // If the robot goes too FAR, make this number BIGGER (like 1.1).
    // If it goes too SHORT, make it SMALLER. (Same idea as the FIRST sample.)
    private static final double GEAR_REDUCTION = 1.0;

    // Wheel size. The StarterBot Hogback wheels are 96 mm across.
    private static final double WHEEL_DIAMETER_MM = 96.0;

    // Distance between the middle of the left wheels and the middle of the
    // right wheels, in inches. Used for turning. Measure your robot!
    private static final double TRACK_WIDTH_INCHES = 12.0;

    // ---- 4. The moves (change these to change the path!) -------------------
    private static final double FIRST_FORWARD_INCHES  = 24.0;  // 2 feet
    private static final double LEFT_TURN_DEGREES     = 90.0;  // quarter turn
    private static final double SECOND_FORWARD_INCHES = 12.0;  // 1 foot

    // ---- 5. Speeds (0.0 = stopped, 1.0 = full speed) ------------------------
    // Slow and steady is more accurate! Start low, speed up once it works.
    private static final double DRIVE_SPEED = 0.45;
    private static final double TURN_SPEED  = 0.35;

    // ---- 6. Safety timeouts (seconds) ---------------------------------------
    // If a move takes longer than this (robot stuck on something?), give up on
    // that move and go on, so the robot never gets stuck forever.
    private static final double DRIVE_TIMEOUT_SECONDS = 5.0;
    private static final double TURN_TIMEOUT_SECONDS  = 4.0;

    // ========================================================================
    //  THINGS THE PROGRAM FIGURES OUT FOR YOU (don't change these)
    // ========================================================================

    // How many wheel inches go by for every one spin of the wheel:
    // distance around a circle = diameter x pi.
    private static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;

    // How many encoder ticks equal 1 inch of driving.
    private static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // These are the "handles" our program uses to talk to the two motors.
    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;

    // A stopwatch, used for the safety timeouts.
    private final ElapsedTime moveTimer = new ElapsedTime();

    // ========================================================================
    //  MAIN PROGRAM  -  runOpMode() runs when you pick this program
    // ========================================================================
    @Override
    public void runOpMode() {

        // ---- STEP A: Get the motors from the robot configuration ----------
        leftDrive  = hardwareMap.get(DcMotor.class, LEFT_DRIVE_NAME);
        rightDrive = hardwareMap.get(DcMotor.class, RIGHT_DRIVE_NAME);

        // ---- STEP B: Set which way each motor spins for "forward" ---------
        leftDrive.setDirection(LEFT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE
                : DcMotor.Direction.FORWARD);
        rightDrive.setDirection(RIGHT_MOTOR_REVERSED ? DcMotor.Direction.REVERSE
                : DcMotor.Direction.FORWARD);

        // BRAKE = when power is 0, the motor holds still instead of coasting.
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- STEP C: Reset the encoders to 0, then turn them on -----------
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- STEP D: Show a message on the Driver Hub and wait for PLAY ---
        telemetry.addData("Status", "Ready! Press PLAY");
        telemetry.addData("Encoder ticks per inch", "%.1f", TICKS_PER_INCH);
        telemetry.addData("Starting at (left : right)", "%d : %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();   // The program pauses here until someone presses PLAY.

        // ---- STEP E: The actual path! -------------------------------------
        driveForward(FIRST_FORWARD_INCHES, "1) Forward 2 ft");
        turnLeft(LEFT_TURN_DEGREES,        "2) Turn left");
        driveForward(SECOND_FORWARD_INCHES, "3) Forward 1 ft");

        // ---- STEP F: Done! ------------------------------------------------
        telemetry.addData("Status", "Path complete");
        telemetry.update();
        sleep(1000);   // Keep the message on screen for 1 second.
    }

    // ========================================================================
    //  EASY-TO-READ MOVES  (these call encoderDrive below)
    // ========================================================================

    /** Drive straight forward. Both sides move the same distance. */
    private void driveForward(double inches, String label) {
        encoderDrive(DRIVE_SPEED, inches, inches, DRIVE_TIMEOUT_SECONDS, label);
    }

    /**
     * Spin in place to the LEFT by a number of degrees.
     * The left wheels go backward and the right wheels go forward.
     *
     * The wheels trace a circle whose width is TRACK_WIDTH_INCHES.
     * A full circle is 360 degrees, so for 90 degrees each wheel travels
     * (pi x track width) x (90 / 360) inches.
     */
    private void turnLeft(double degrees, String label) {
        double wheelInches = Math.PI * TRACK_WIDTH_INCHES * (degrees / 360.0);
        encoderDrive(TURN_SPEED, -wheelInches, wheelInches, TURN_TIMEOUT_SECONDS, label);
    }

    // ========================================================================
    //  THE WORKHORSE: encoderDrive  (same idea as the official FIRST sample)
    // ========================================================================

    /**
     * Move each side a certain number of inches, then stop.
     *
     * @param speed         how fast to go (0 to 1). Always positive!
     * @param leftInches    how far the LEFT side moves (negative = backward)
     * @param rightInches   how far the RIGHT side moves (negative = backward)
     * @param timeoutSeconds give up after this many seconds (safety)
     * @param label         a name shown on the Driver Hub so you know the step
     *
     * Note: to go backward we use a NEGATIVE DISTANCE, not a negative speed.
     *
     * The move ends when ANY of these happens:
     *   1) the robot reaches the target distance
     *   2) the timeout runs out
     *   3) someone presses STOP on the Driver Hub
     */
    private void encoderDrive(double speed, double leftInches, double rightInches,
                              double timeoutSeconds, String label) {

        // Only move if the program is still running (nobody pressed STOP).
        if (!opModeIsActive()) {
            return;
        }

        // 1) Figure out the target: where the motors are NOW + how far to go.
        //    (Using "now + distance" means each move starts from wherever the
        //    last move ended - no need to reset the encoders every time.)
        int newLeftTarget  = leftDrive.getCurrentPosition()  + (int) (leftInches  * TICKS_PER_INCH);
        int newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);

        // 2) Tell each motor its target. This must be done BEFORE RUN_TO_POSITION.
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // 3) "RUN_TO_POSITION" = the motor controller drives to the target and
        //    holds there by itself.
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 4) Start the stopwatch and GO!
        moveTimer.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // 5) Keep waiting while: the program is running AND time is left AND
        //    both motors are still moving. "isBusy()" means "not at target yet".
        //    (&& = "and". If either motor arrives, the move ends. That is the
        //    safer choice and is what the official sample does.)
        while (opModeIsActive()
                && moveTimer.seconds() < timeoutSeconds
                && leftDrive.isBusy() && rightDrive.isBusy()) {

            telemetry.addData("Step", label);
            telemetry.addData("Target (left : right)", "%d : %d", newLeftTarget, newRightTarget);
            telemetry.addData("Now    (left : right)", "%d : %d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.update();
        }

        // 6) Stop both motors.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // 7) Switch back to normal encoder mode, ready for the next move.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 8) A short pause so the robot settles before the next move.
        sleep(250);
    }
}