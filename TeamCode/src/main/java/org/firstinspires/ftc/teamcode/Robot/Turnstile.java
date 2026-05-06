package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Auto.AutoRedDepot_12;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config // Make this class tunable
public class Turnstile {


    public static double SECOND_BALL_BOOST = 1.1;
    public static double BALL2_ANGLE = 240;
    public static double BALL3_ANGLE = 120;
    public static double THIRD_BALL_BOOST = 1.2;
    // --- Hardware & Utilities ---
    private CRServo indexerServo1;
    private CRServo indexerServo2;
    private DcMotorEx indexMotor;
    private TouchSensor limitSwitch;
    private PIDController angleController;
    private Telemetry telemetry;

    public static boolean TELEM = false;

    // --- Tunable Constants via FTC Dashboard ---
    public static double P = 0.0028, I = 0.0, D = 0.00022;
    public static double P_PER_BALL_FACTOR = 0.00029;
    public static double THRESHOLD = 0.00;
    public static double MIN_POWER_POS = 0.035;
    public static double MIN_POWER_NEG = 0.0;
    public static double HOMING_POWER = 0.065;
    public static double ANGLE_TOLERANCE = 12.5;// In degrees
    public static double CYCLE_TIME = 50;
    public static double BACKWARD_TOLERANCE = 30;
    public static double INTAKE_TOLERANCE = 10;
    public static double HOMING_OFFSET = 0;
    public static double LAUNCHING_POWER = 0.95;
    private double current_offset = 0; // --- Non-tunable Constants ---
    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double GEAR_RATIO = (double) 1 / 2;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION / 360;
    public static final String STARTING_ANGLE_KEY = AutoRedDepot_12.ENDING_ANGLE_INDEXER_KEY;
    public double startingAngle;
    private double launching_offset = 0;
    public static double LAUNCHING_OFFSET_ANGLE = 15;
    public static double INTAKE_OFFSET_ANGLE = -5;
    private boolean nearTarget;
    public ElapsedTime debounce = new ElapsedTime();
    private int numLaunches = 3;


    // --- State Management ---
    public enum State {IDLE, HOMING, SEEKING_POSITION, HOLDING_POSITION, MANUAL_SPIN, LAUNCHING} // Added MANUAL_SPIN

    private State currentState = State.IDLE;
    private double targetAngle = 0;
    private double manualPower = 0; // For spin()
    private boolean isHomed = false;


    // --- Cached Hardware Values ---
    private double currentAngle;
    private boolean limitSwitchPressed;

    public void init(HardwareMap hwMap, Telemetry telem) {
        telemetry = telem;
        angleController = new PIDController(P, I, D);


        try {
            indexerServo1 = hwMap.crservo.get("indexer");
            indexerServo2 = hwMap.crservo.get("indexer2");

//            indexerServo1.setDirection(DcMotorSimple.Direction.REVERSE);
//            indexerServo2.setDirection(DcMotorSimple.Direction.REVERSE);

            indexMotor = hwMap.get(DcMotorEx.class, MecanumDrive.LEFT_FRONT_MOTOR);
            limitSwitch = hwMap.get(TouchSensor.class, "indexerLimit");

//            indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Use our own P
            indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use our own PID
        } catch (RuntimeException e) {
            telemetry.addData("Turnstile Hardware Not Found", e.getMessage());
        }
        startingAngle = (double) blackboard.getOrDefault(STARTING_ANGLE_KEY, (double) 0);
    }

    // --- High-Level Commands ---

    public void home() {
        currentState = State.HOMING;
        isHomed = false;
    }

    public void intakeStart() {
        launching_offset = INTAKE_OFFSET_ANGLE;
    }

    public void intakeStop() {
        launching_offset = LAUNCHING_OFFSET_ANGLE;
    }

    public void launchSlots(int launches) {
        targetAngle = currentAngle - (120 * launches);
        numLaunches = launches;

        currentState = State.LAUNCHING;
    }

    public void seekToAngle(double angle) {

        angle = ((angle % 360) + 360) % 360; // Make sure angle is within [0, 360]

        // Shortest path shortestRot in [-180,180]
        double shortestRot = angle - (currentAngle % 360.0);
        shortestRot = ((shortestRot + 180) % 360) - 180;

        // If shortestRot is too far behind, force forward rotation
        if (shortestRot < -ANGLE_TOLERANCE && Math.abs(shortestRot) > BACKWARD_TOLERANCE) {
            shortestRot += 360.0;
        }

        targetAngle = currentAngle + shortestRot;


        currentState = State.SEEKING_POSITION;
        nearTarget = false;
        /* --- Old Implementation ---
        // Refactored to have a single exit point
        //if (isHomed) {
            this.targetAngle = angle;
            this.currentState = State.SEEKING_POSITION;
        //}
        */
    }

    public void spin(double power) {
        // Refactored to have a single exit point
        //if (isHomed) {

        if (0 == Math.abs(power) && State.SEEKING_POSITION != currentState && State.HOLDING_POSITION != currentState) {
            // When driver lets go, find the nearest physical slot and seek to it.
            double nearestSlotAngle = (Math.round(currentAngle / 120.0)) * 120.0;
            seekToAngle(nearestSlotAngle);
            currentState = State.SEEKING_POSITION;
        } else if (0 < Math.abs(power)) {
            manualPower = power;
            currentState = State.MANUAL_SPIN;
        } else {
            currentState = State.SEEKING_POSITION;
        }

        //}
    }

    // --- State Inquiry ---

    public boolean isAtTarget() {
        return (Math.abs(currentAngle - (targetAngle + current_offset)) < ANGLE_TOLERANCE);
    }

    public boolean isAtTargetTime() {
        if (nearTarget) {
            return isTimerReady();
        } else {
            debounce.reset();
            nearTarget = true;
            return false;
        }
    }

    public boolean isTimerReady() {
        return (CYCLE_TIME < debounce.milliseconds());
    }

    public boolean isOverSlot() {
        return Math.abs(currentAngle - (targetAngle + current_offset)) < INTAKE_TOLERANCE;
    }

    public boolean isHomed() {
        return isHomed;
    }


    double getCurrentAngle() {
        return currentAngle;
    }

    public void update(int count) {
        // --- 1. Cache Hardware Reads ---
        currentAngle = indexMotor.getCurrentPosition() / COUNTS_PER_DEGREE - startingAngle - launching_offset;
        double angleErrorAbs = Math.abs(targetAngle + current_offset - currentAngle);
        double lowerErrorScalar = (angleErrorAbs * angleErrorAbs) / (ANGLE_TOLERANCE * ANGLE_TOLERANCE);
        limitSwitchPressed = limitSwitch.isPressed();

        int minCount = Math.min(count, 3);
        // --- 2. Run State Machine ---
        double power;
        double pTotal = P + P_PER_BALL_FACTOR * minCount;
        angleController.setPID(pTotal, I, D); // Re-apply PID gains from Dashboard
        power = angleController.calculate(currentAngle, targetAngle + current_offset) * GEAR_RATIO;

        if (power > THRESHOLD) power = Range.scale(power, THRESHOLD, 1, MIN_POWER_POS, 1);
        if (power < -THRESHOLD) power = Range.scale(power, -1, -THRESHOLD, -1, -MIN_POWER_NEG);

        switch (currentState) {
            case IDLE:
                indexerServo1.setPower(0);
                indexerServo2.setPower(0);
                break;

            case HOMING:
                if (limitSwitchPressed) {
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentAngle = 0;
                    isHomed = true;
                    targetAngle = 0;
                    current_offset = HOMING_OFFSET;

                    indexerServo1.setPower(power * lowerErrorScalar);
                    indexerServo2.setPower(power * lowerErrorScalar);
                    currentState = State.HOLDING_POSITION;
                } else {
                    indexerServo1.setPower(HOMING_POWER);
                    indexerServo2.setPower(HOMING_POWER);
                    isHomed = false;
                }
                break;

            case MANUAL_SPIN:
                indexerServo1.setPower(manualPower);
                indexerServo2.setPower(manualPower);
                if (0.01 > Math.abs(manualPower)) {
                    // When driver lets go, find the nearest physical slot and seek to it.
                    double nearestSlotAngle = (Math.round(currentAngle / 120.0)) * 120.0;
                    seekToAngle(nearestSlotAngle);
                    currentState = State.SEEKING_POSITION;
                }
                break;

            case SEEKING_POSITION:
                if (isAtTarget()) {
                    currentState = State.HOLDING_POSITION;
                    // We have arrived. Stop the motor for this one cycle to prevent a "kick".
                    // The next loop will execute the HOLDING_POSITION logic.
                    indexerServo1.setPower(power * lowerErrorScalar);
                    indexerServo2.setPower(power * lowerErrorScalar);
                } else {
                    // If not at target, continue seeking.
                    indexerServo1.setPower(power);
                    indexerServo2.setPower(power);
                }
                break;
            case LAUNCHING:
                if (currentAngle <= targetAngle) {
                    currentState = State.SEEKING_POSITION;
                    // We have arrived. Stop the motor for this one cycle to prevent a "kick".
                    // The next loop will execute the HOLDING_POSITION logic.
                    indexerServo1.setPower(0);
                    indexerServo2.setPower(0);
                } else if (3 == numLaunches && currentAngle > (targetAngle - BALL3_ANGLE) && currentAngle <= targetAngle - BALL2_ANGLE) {
                    double fasterPower = Range.clip(-LAUNCHING_POWER * SECOND_BALL_BOOST, -1, 1);
                    indexerServo1.setPower(fasterPower);
                    indexerServo2.setPower(fasterPower);
                } else if (3 == numLaunches && currentAngle <= targetAngle - BALL3_ANGLE) {
                    // If not at target, continue seeking.
                    double fasterPower = Range.clip(-LAUNCHING_POWER * THIRD_BALL_BOOST, -1, 1);
                    indexerServo1.setPower(fasterPower);
                    indexerServo2.setPower(fasterPower);
                } else {

                    // If not at target, continue seeking.
                    indexerServo1.setPower(-LAUNCHING_POWER);
                    indexerServo2.setPower(-LAUNCHING_POWER);
                }
                break;

            case HOLDING_POSITION:
                // If a magnet is detected while holding, we use it to correct for encoder drift.
                if (isAtTarget()) {
                    indexerServo1.setPower(power / 2);
                    indexerServo2.setPower(power / 2);
                } else {
                    indexerServo1.setPower(power);
                    indexerServo2.setPower(power);
                    currentState = State.SEEKING_POSITION;
                }
                break;
        }

        // --- 3. Telemetry ---
        if (TELEM) {
//        telemetry.addData("Turnstile State", currentState.name());
            telemetry.addData("Turnstile Angle", currentAngle);
            telemetry.addData("Turnstile Target", targetAngle + current_offset);
            telemetry.addData("Turnstile Error: ", targetAngle + current_offset - currentAngle);
            telemetry.addData("Turnstile Power", power);
            telemetry.addData("Limit Switch Pressed", limitSwitchPressed);
        }
    }
}
