/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.test;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/*
 * =====================================================================================
 *  BioBuzz StarterBot - Driver-Controlled (TeleOp) Program
 * =====================================================================================
 *
 * WHAT IS THIS FILE?
 *   This is the program a driver runs during the "driver-controlled" part of an FTC match.
 *   The driver uses a gamepad, and this code turns the gamepad buttons and sticks into
 *   robot movement.
 *
 * WHAT CAN THE ROBOT DO?
 *   1. DRIVE   - Two drive motors (left and right) move the robot around (a "skid-steer"
 *                robot, like a tank).
 *   2. INTAKE  - One motor and two servos pull game elements (balls) into the robot.
 *   3. LAUNCH  - A fast spinning wheel (the launcher) throws the elements. A "windmill"
 *                servo pushes the elements into the launcher wheel.
 *
 * GAMEPAD CONTROLS (gamepad1)
 *   Left stick up/down   -> drive forward / backward
 *   Right stick left/right -> turn left / right
 *   Right trigger        -> intake pulls elements IN
 *   Left trigger         -> intake pushes elements OUT (reverse)
 *   Right bumper (hold)  -> spin up the launcher, then feed elements when it is fast enough
 *
 * HOW THIS FILE IS ORGANIZED (read from top to bottom)
 *   1. Tuning constants   - numbers you can change to adjust the robot (START HERE!)
 *   2. Hardware           - the motors and servos the robot has
 *   3. Robot state        - variables that remember values while the program runs
 *   4. OpMode lifecycle   - init(), init_loop(), start(), loop(), stop()
 *   5. Helper methods     - arcadeDrive() and launch()
 *
 * ADVANCED IDEA - "CLOSED-LOOP" VELOCITY CONTROL
 *   The launcher uses "RUN_USING_ENCODER" mode. An encoder is a sensor that counts how
 *   fast the motor is spinning. The motor controller keeps comparing the real speed to
 *   the target speed and adds or removes power to hold the target. This is called
 *   closed-loop control. It works like cruise control in a car!
 *   The "PIDF" numbers below tell the controller how strongly to react.
 */

@TeleOp(name = "BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterBotTeleopClaud extends OpMode {

    // =================================================================================
    // 1. TUNING CONSTANTS
    //    Change these numbers to adjust how the robot behaves. You do NOT need to
    //    touch the rest of the code. "final" means the value never changes while the
    //    program runs. "static" means the value belongs to the class itself.
    // =================================================================================

    // ---------- Hardware names ----------
    // These names MUST match exactly what you typed in the Robot Configuration on the
    // Driver Station phone/Control Hub (including capital letters and underscores).
    // If a name is wrong, the program will crash on INIT with a "device not found" error.
    private static final String LEFT_DRIVE_NAME = "left_drive";
    private static final String RIGHT_DRIVE_NAME = "right_drive";
    private static final String INTAKE_NAME = "intake";
    private static final String LAUNCHER_NAME = "launcher";
    private static final String WINDMILL_NAME = "windmill";
    private static final String LEFT_INTAKE_SERVO_NAME = "left_intake_servo";
    private static final String RIGHT_INTAKE_SERVO_NAME = "right_intake_servo";

    // ---------- Motor and servo directions ----------
    // The motors on the left and right sides face opposite ways, so one side must be
    // reversed. Pushing the left stick forward MUST make the robot go forward.
    // If your robot drives backward or spins in a circle, swap FORWARD and REVERSE here.
    // (Gear reductions or 90-degree drives may also need these flipped.)
    private static final DcMotor.Direction LEFT_DRIVE_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_DRIVE_DIRECTION = DcMotor.Direction.REVERSE;

    // The right intake servo is mounted facing the opposite way from the left one, so we
    // reverse it. This makes BOTH servos pull elements in at the same time.
    private static final DcMotorSimple.Direction RIGHT_INTAKE_SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ---------- Launcher speed ----------
    // Speeds are measured in "encoder ticks per second". FTC motors have an encoder with
    // 28 ticks for every full turn of the motor.
    //   To convert to RPM (turns per minute):  RPM = ticks per second / 28 * 60
    //   Example: 1250 ticks/sec = 1250 / 28 * 60 = about 2679 RPM
    public static final int LAUNCHER_TARGET_VELOCITY = 1300; // the speed we WANT the launcher to reach
    public static final int LAUNCHER_MIN_VELOCITY = 1200;    // the slowest speed that still makes a good shot
    public static final int LAUNCHER_OFF_VELOCITY = 0;       // speed when the driver is not launching

    // The windmill only runs when the launcher is faster than LAUNCHER_MIN_VELOCITY.
    // This way elements are never fed into a wheel that is too slow, which would make a
    // weak, short shot. Keep MIN a little below TARGET so the windmill doesn't flicker
    // on and off.

    // ---------- Launcher PIDF tuning ----------
    // These four numbers control the closed-loop "cruise control" for the launcher wheel.
    //   P (Proportional): how hard to push when the speed is wrong. Higher = reacts faster.
    //   I (Integral)    : fixes small errors that last a long time. We don't need it (0).
    //   D (Derivative)  : calms the motor down to stop overshooting. We don't need it (0).
    //   F (Feedforward) : the "base" power guess for a target speed. This does most of the work.
    // Tip: Change only ONE number at a time, then test, so you know what helped.
    private static final double LAUNCHER_PID_P = 40;
    private static final double LAUNCHER_PID_I = 0;
    private static final double LAUNCHER_PID_D = 0;
    private static final double LAUNCHER_PID_F = 12.5;

    // ---------- Feeding and intake power ----------
    // Servo/motor power goes from -1.0 (full reverse) to +1.0 (full forward). 0 is stopped.
    private static final double WINDMILL_FEED_POWER = 1.0;  // windmill power while feeding the launcher
    private static final double WINDMILL_OFF_POWER = 0.0;   // windmill power when stopped
    private static final double INTAKE_STOP_POWER = 0.0;    // used to wake up the servos at INIT
    // While launching, we add extra intake power. This helps shake loose any element
    // that is stuck inside the hopper (the storage area).
    private static final double LAUNCH_INTAKE_BOOST = 0.5;

    // ---------- Driving ----------
    // Multiplies the drive and turn inputs. 1.0 = full speed. Try 0.5 for "slow mode"
    // while learning to drive. (Using 1.0 keeps the original behavior.)
    private static final double DRIVE_SPEED_SCALE = 1.0;
    private static final double TURN_SPEED_SCALE = 1.0;

    // ---------- Telemetry (information shown on the Driver Station) ----------
    // "Telemetry" means the robot sends numbers back to the Driver Station screen so you
    // can see what is happening inside the program. It is the best tool for finding bugs!
    //
    // Master switch. Set to false to turn ALL telemetry off (the robot then runs exactly
    // like the original program, with no extra hardware reads).
    private static final boolean TELEMETRY_ENABLED = true;

    // false = short screen (about 6 lines + warnings) - good for matches and driving.
    // true  = adds gamepad values, encoders, motor current, PIDF values, and more -
    //         turn this on when you are troubleshooting or tuning.
    private static final boolean SHOW_DETAILED_TELEMETRY = false;

    // How often (in milliseconds) telemetry is sent to the Driver Station.
    // 250 = four updates per second. Smaller = faster updates but more data traffic.
    private static final int TELEMETRY_INTERVAL_MS = 250;

    // The motor encoder counts this many ticks per full turn (used to compute RPM).
    private static final double ENCODER_TICKS_PER_REVOLUTION = 28.0;

    // ---------- Warning limits ----------
    // If a value crosses one of these limits, a WARNING line appears on the screen.
    // Battery voltage below this value may make motors weak and launcher speed drop.
    private static final double LOW_BATTERY_VOLTS = 11.5;
    // If the launcher goes this many ticks/sec ABOVE the target, something is probably wrong
    // (usually the encoder cable). See setupLauncher() for troubleshooting tips.
    private static final int LAUNCHER_OVERSHOOT_WARNING = 300;
    // If the loop takes longer than this many milliseconds, the program is running slowly.
    private static final double SLOW_LOOP_WARNING_MS = 50.0;
    // Below this speed (ticks/sec) the launcher wheel is considered stopped.
    private static final int LAUNCHER_STOPPED_VELOCITY = 50;

    // =================================================================================
    // 2. HARDWARE
    //    These variables are the "handles" to the real motors and servos on the robot.
    //    They start as null (empty) and are filled in during init().
    // =================================================================================

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;     // "Ex" = extra features, like setVelocity()
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;  // CRServo = Continuous Rotation Servo (spins like a motor)
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // =================================================================================
    // 3. ROBOT STATE
    //    Variables that remember values while the program runs. We keep these here (not
    //    inside the methods) so loop() can show them on the Driver Station screen.
    // =================================================================================

    double leftPower;    // power we are sending to the left drive motor
    double rightPower;   // power we are sending to the right drive motor
    double intakePower;  // power we are sending to the intake motor and servos

    // Helpers used only for telemetry.
    private VoltageSensor batteryVoltageSensor = null; // reads the robot battery voltage
    private final ElapsedTime loopTimer = new ElapsedTime(); // measures how long each loop takes
    private PIDFCoefficients launcherPidf = null;            // launcher PIDF values read from the hub

    // =================================================================================
    // 4. OPMODE LIFECYCLE
    //    The FTC system calls these methods for you, in this order:
    //      init() -> init_loop() (repeats) -> start() -> loop() (repeats) -> stop()
    // =================================================================================

    /*
     * Runs ONCE when the driver presses INIT.
     * This is where we connect to the hardware and set it up.
     */
    @Override
    public void init() {
        setupHardware();
        setupDrive();
        setupLauncher();
        setupIntake();

        // Control how often telemetry is sent to the Driver Station.
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);

        // Read the PIDF values the hub is really using, ONCE, so telemetry can show them
        // without asking the hub again on every loop. If this fails, the robot still works.
        if (TELEMETRY_ENABLED) {
            try {
                launcherPidf = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (RuntimeException e) {
                launcherPidf = null;
            }
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized - press START when ready");
    }

    /*
     * Runs over and over after INIT but before START.
     * We use it to show the settings that were loaded, so you can check them BEFORE
     * the match starts. If a value looks wrong, change the constants at the top of the file.
     */
    @Override
    public void init_loop() {
        if (!TELEMETRY_ENABLED) {
            return;
        }
        double battery = getBatteryVoltage();
        telemetry.addData("Status", "Initialized - press START when ready");
        telemetry.addData("Battery", "%.2f V%s", battery,
                battery < LOW_BATTERY_VOLTS ? "  <-- LOW! Swap the battery" : "  (OK)");
        telemetry.addData("Drive directions", "left=%s, right=%s",
                LEFT_DRIVE_DIRECTION, RIGHT_DRIVE_DIRECTION);
        telemetry.addData("Drive scale", "drive=%.2f, turn=%.2f", DRIVE_SPEED_SCALE, TURN_SPEED_SCALE);
        telemetry.addData("Launcher target / min", "%d / %d ticks/sec (%.0f RPM target)",
                LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY, ticksPerSecToRpm(LAUNCHER_TARGET_VELOCITY));
        if (launcherPidf != null) {
            telemetry.addData("Launcher PIDF", "P=%.2f I=%.2f D=%.2f F=%.2f",
                    launcherPidf.p, launcherPidf.i, launcherPidf.d, launcherPidf.f);
        }
        telemetry.addData("Intake boost while launching", "%.2f", LAUNCH_INTAKE_BOOST);
    }

    /*
     * Runs ONCE when the driver presses START.
     * We only use it to restart the loop timer (telemetry only - it does not affect the robot).
     */
    @Override
    public void start() {
        // Start timing from zero so the first loop is not counted as "slow" (INIT can take a while).
        loopTimer.reset();
    }

    /*
     * Runs over and over (about 50+ times a second) after START, until STOP is pressed.
     * Each pass through loop() reads the gamepad and updates the robot.
     */
    @Override
    public void loop() {
        // Step 1: Drive the robot using the sticks.
        // The left stick Y is NEGATIVE when pushed up, so we flip it with a minus sign.
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Step 2: Work out the intake power from the triggers.
        // Each trigger gives a number from 0 (released) to 1 (fully pressed).
        //   Right trigger only -> positive power (pull elements IN)
        //   Left trigger only  -> negative power (push elements OUT)
        //   Both triggers      -> they cancel out, so the intake stays off
        // We save the result in a variable first and set the motors only ONCE per loop.
        // Setting the same motor more than once per loop can cause jerky behavior.
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Step 3: Handle the launcher.
        // IMPORTANT: launch() may change intakePower (it adds a boost), so it must run
        // BEFORE we send intakePower to the motors in the next step.
        launch();

        // Step 4: Send the final intake power to the intake motor and both servos.
        intake.setPower(intakePower);
        leftIntakeServo.setPower(-intakePower);
        rightIntakeServo.setPower(-intakePower);

        // Step 5: Show information on the Driver Station screen (telemetry).
        updateTelemetry();
    }

    /*
     * Runs ONCE when the driver presses STOP. We don't need it, so it is empty.
     */
    @Override
    public void stop() {
    }

    // =================================================================================
    // 5. SETUP HELPERS (called from init)
    //    Splitting setup into small methods keeps init() short and easy to read.
    // =================================================================================

    /*
     * Connects each variable to the real device using the names from the robot
     * configuration. The names are the constants defined at the top of this file.
     */
    private void setupHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, LEFT_DRIVE_NAME);
        rightDrive = hardwareMap.get(DcMotor.class, RIGHT_DRIVE_NAME);
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
        windmillServo = hardwareMap.get(CRServo.class, WINDMILL_NAME);
        leftIntakeServo = hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO_NAME);
        rightIntakeServo = hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO_NAME);

        // The battery voltage sensor is built into the Control Hub / Expansion Hub.
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
    }

    /*
     * Sets the drive motor directions, and turns on BRAKE mode.
     * BRAKE mode makes a motor stop quickly when its power is 0 (instead of slowly
     * coasting). That makes the robot much easier to control.
     */
    private void setupDrive() {
        leftDrive.setDirection(LEFT_DRIVE_DIRECTION);
        rightDrive.setDirection(RIGHT_DRIVE_DIRECTION);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
    }

    /*
     * Puts the launcher in RUN_USING_ENCODER mode (closed-loop speed control) and loads
     * our PIDF tuning numbers.
     * TROUBLESHOOTING: If the launcher jumps to a speed much higher than the target,
     * check that the encoder cable is plugged into the port right next to the motor,
     * and that the motor wiring polarity is correct.
     */
    private void setupLauncher() {
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_PID_P, LAUNCHER_PID_I, LAUNCHER_PID_D, LAUNCHER_PID_F));
    }

    /*
     * Sets up the intake motor and the servos that feed elements.
     */
    private void setupIntake() {
        intake.setZeroPowerBehavior(BRAKE);

        // Send a power of 0 once so the servo controller wakes up and is ready.
        leftIntakeServo.setPower(INTAKE_STOP_POWER);
        rightIntakeServo.setPower(INTAKE_STOP_POWER);
        windmillServo.setPower(WINDMILL_OFF_POWER);

        // Reverse the mirrored servos so that they all spin the correct way.
        rightIntakeServo.setDirection(RIGHT_INTAKE_SERVO_DIRECTION);
        windmillServo.setDirection(WINDMILL_DIRECTION);
    }

    // =================================================================================
    // 6. ROBOT ACTIONS (called from loop)
    // =================================================================================

    /*
     * ARCADE DRIVE
     * "Arcade" is the name of this control style, like an old arcade game:
     *   - Push the left stick forward  -> both wheels go forward.
     *   - Push the right stick sideways -> the wheels go opposite ways, so the robot turns.
     *   - Mix both at once to drive in a curve.
     *
     * The math: to turn right, the left wheel goes faster and the right wheel slower.
     *   leftPower  = forward + rotate
     *   rightPower = forward - rotate
     *
     * @param forward how much to move forward (-1 backward ... +1 forward)
     * @param rotate  how much to turn (-1 left ... +1 right)
     */
    void arcadeDrive(double forward, double rotate) {
        leftPower = (forward * DRIVE_SPEED_SCALE) + (rotate * TURN_SPEED_SCALE);
        rightPower = (forward * DRIVE_SPEED_SCALE) - (rotate * TURN_SPEED_SCALE);

        // Send the calculated power to the motors.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


    /*
     * LAUNCH
     * Controls the launcher wheel and the windmill servo that feeds it.
     *
     * Holding the RIGHT BUMPER does two things:
     *   1. Spins the launcher wheel up to LAUNCHER_TARGET_VELOCITY.
     *   2. Once the wheel is fast enough (above LAUNCHER_MIN_VELOCITY), runs the windmill
     *      servo to push elements into the wheel, and adds a little extra intake power.
     *
     * Letting go of the bumper stops both. (gamepad1.right_bumper is true on every loop
     * pass while the button is held down, and false otherwise.)
     */
    void launch() {
        boolean launchButtonHeld = gamepad1.right_bumper;

        // Part 1: spin the launcher up when the button is held, spin it down when not.
        if (launchButtonHeld) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(LAUNCHER_OFF_VELOCITY);
        }

        // Part 2: only feed elements when the button is held AND the wheel is fast enough.
        // The "&&" means "and": both things must be true.
        if (launchButtonHeld && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(WINDMILL_FEED_POWER);
            intakePower += LAUNCH_INTAKE_BOOST; // extra push to free stuck elements
        } else {
            windmillServo.setPower(WINDMILL_OFF_POWER);
        }
    }

    // =================================================================================
    // 7. TELEMETRY HELPERS
    //    These methods only READ values and show them. They never move the robot.
    // =================================================================================

    /*
     * Shows telemetry on the Driver Station. This is "read only": it never changes how the
     * robot moves. It runs at the very END of loop(), after all motor commands were sent.
     * If anything goes wrong while reading values for the screen, we catch the error so
     * that telemetry can NEVER stop the robot from driving.
     */
    private void updateTelemetry() {
        if (!TELEMETRY_ENABLED) {
            return;
        }
        try {
            showTelemetry();
        } catch (RuntimeException e) {
            telemetry.addLine("Telemetry problem: " + e.getMessage());
        }
    }

    /*
     * SHORT SCREEN (always shown):
     *   battery and loop time, drive power, intake power, launcher state and speed,
     *   and a WARNING line only when something looks wrong.
     * DETAILED SCREEN (only if SHOW_DETAILED_TELEMETRY is true) adds:
     *   run time, gamepad values, windmill and intake servo power, drive encoders,
     *   launcher current, PIDF values and launcher error.
     */
    private void showTelemetry() {
        // ----- How long did this loop take? -----
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        double battery = getBatteryVoltage();
        double launcherVelocity = launcher.getVelocity();
        int launcherTargetVelocity = gamepad1.right_bumper ? LAUNCHER_TARGET_VELOCITY : LAUNCHER_OFF_VELOCITY;
        boolean launcherOverCurrent = false; // only checked when detailed telemetry is on

        // ----- SHORT SCREEN -----
        telemetry.addData("Battery / loop", "%.1f V / %.0f ms", battery, loopMs);
        telemetry.addData("Drive power (L / R)", "%.2f / %.2f", leftPower, rightPower);
        telemetry.addData("Intake power", "%.2f", intakePower);
        // Compare "actual" with "target". If actual stays far from target, tune the PIDF
        // numbers (mostly F, then P) at the top of the file.
        telemetry.addData("Launcher", "%s", getLauncherState(launcherVelocity));
        telemetry.addData("Launcher speed", "%.0f / %d ticks/sec (%.0f RPM)",
                launcherVelocity, launcherTargetVelocity, ticksPerSecToRpm(launcherVelocity));

        // ----- DETAILED SCREEN (extra information for tuning and troubleshooting) -----
        if (SHOW_DETAILED_TELEMETRY) {
            telemetry.addLine("--- details ---");
            telemetry.addData("Run time", "%.1f sec", getRuntime());

            // If these numbers never change when you move the controls, the gamepad is not
            // connected, or it is not assigned as "User 1" on the Driver Station.
            telemetry.addData("Sticks (fwd / turn)", "%.2f / %.2f", -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Triggers (R / L)", "%.2f / %.2f", gamepad1.right_trigger, gamepad1.left_trigger);
            telemetry.addData("Right bumper", gamepad1.right_bumper ? "PRESSED" : "not pressed");

            telemetry.addData("Windmill power", "%.2f", windmillServo.getPower());
            telemetry.addData("Intake servos (L / R)", "%.2f / %.2f",
                    leftIntakeServo.getPower(), rightIntakeServo.getPower());
            telemetry.addData("Min speed to feed", "%d ticks/sec", LAUNCHER_MIN_VELOCITY);
            telemetry.addData("Launcher error", "%.0f ticks/sec (target - actual)",
                    launcherTargetVelocity - launcherVelocity);

            // Encoder positions go UP when a wheel turns forward. If one side counts down
            // while driving forward, that motor's direction constant needs to be flipped.
            telemetry.addData("Drive encoders (L / R)", "%d / %d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());

            // Current is how much electricity a motor uses (in amps). A very high number
            // for a long time means the motor is stuck, jammed, or overloaded.
            launcherOverCurrent = launcher.isOverCurrent();
            telemetry.addData("Launcher current", "%.2f A%s", launcher.getCurrent(CurrentUnit.AMPS),
                    launcherOverCurrent ? "  <-- OVER CURRENT!" : "");

            if (launcherPidf != null) {
                telemetry.addData("Launcher PIDF", "P=%.2f I=%.2f D=%.2f F=%.2f",
                        launcherPidf.p, launcherPidf.i, launcherPidf.d, launcherPidf.f);
            }
        }

        // ----- WARNINGS (only appear when something looks wrong) -----
        // (addLine is used so several warnings can show at once without sharing a caption.)
        if (battery > 0 && battery < LOW_BATTERY_VOLTS) {
            telemetry.addLine("WARNING: Low battery! Motors will be weak. Swap the battery.");
        }
        if (gamepad1.right_bumper && launcherVelocity > LAUNCHER_TARGET_VELOCITY + LAUNCHER_OVERSHOOT_WARNING) {
            telemetry.addLine("WARNING: Launcher way too fast! Check the encoder cable and motor wiring.");
        }
        if (launcherOverCurrent) {
            telemetry.addLine("WARNING: Launcher motor over current! Is it jammed?");
        }
        if (loopMs > SLOW_LOOP_WARNING_MS) {
            telemetry.addLine("WARNING: Slow loop (" + Math.round(loopMs) + " ms). Is something slow inside loop()?");
        }
    }

    /*
     * Returns a short, readable description of what the launcher is doing right now.
     */
    private String getLauncherState(double actualVelocity) {
        if (!gamepad1.right_bumper) {
            return actualVelocity > LAUNCHER_STOPPED_VELOCITY ? "SPINNING DOWN" : "OFF";
        }
        return actualVelocity > LAUNCHER_MIN_VELOCITY ? "READY - FEEDING ELEMENTS" : "SPINNING UP";
    }

    /*
     * Converts encoder ticks per second into RPM (revolutions per minute).
     * Example: 1250 ticks/sec / 28 ticks per turn * 60 seconds = about 2679 RPM.
     */
    private double ticksPerSecToRpm(double ticksPerSecond) {
        return ticksPerSecond / ENCODER_TICKS_PER_REVOLUTION * 60.0;
    }

    /*
     * Returns the battery voltage, or 0 if no voltage sensor was found.
     */
    private double getBatteryVoltage() {
        return batteryVoltageSensor != null ? batteryVoltageSensor.getVoltage() : 0.0;
    }
}