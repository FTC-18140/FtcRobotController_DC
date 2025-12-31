package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config // Make this class tunable
public class Turnstile {

    // --- Hardware & Utilities ---
    private CRServo indexerServo1;
    private CRServo indexerServo2;
    private DcMotorEx indexMotor;
    private TouchSensor limitSwitch;
    private PIDController angleController;
    private Telemetry telemetry;

    // --- Tunable Constants via FTC Dashboard ---
    public static double P = 0.0040, I = 0.0007, D = 0.0001;
    public static double HOMING_POWER = -0.05;
    public static double ANGLE_TOLERANCE = 13;// In degrees
    public static double BACKWARD_TOLERANCE = 30;
    public static double HOMING_OFFSET = 15;
    private double current_offset = 0; // --- Non-tunable Constants ---
    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double GEAR_RATIO = 1.0;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * GEAR_RATIO)/360;
    public static final String STARTING_ANGLE_KEY = "ENDING_ANGLE_INDEXER";
    public double startingAngle;

    // --- State Management ---
    public enum State { IDLE, HOMING, SEEKING_POSITION, HOLDING_POSITION, MANUAL_SPIN } // Added MANUAL_SPIN
    private State currentState = State.IDLE;
    private double targetAngle = 0;
    private double manualPower = 0; // For spin()
    private boolean isHomed = false;


    // --- Cached Hardware Values ---
    private double currentAngle;
    private boolean limitSwitchPressed;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        angleController = new PIDController(P, I, D);


        try {
            indexerServo1 = hwMap.crservo.get("indexer");
            indexerServo2 = hwMap.crservo.get("indexer2");
            indexMotor = hwMap.get(DcMotorEx.class, "indexMotor");
            limitSwitch = hwMap.get(TouchSensor.class, "indexerLimit");

            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Use our own P
            indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use our own PID
        } catch (Exception e) {
            telemetry.addData("Turnstile Hardware Not Found", e.getMessage());
        }
        startingAngle = (double) blackboard.getOrDefault(STARTING_ANGLE_KEY, (double) 0);
    }

    // --- High-Level Commands ---

    public void home() {
        if (!isHomed) {
            currentState = State.HOMING;
        }
    }

    public void seekToAngle(double angle) {

        angle = ((angle % 360) + 360) % 360; // Make sure angle is within [0, 360]

        // Shortest path shortest_rot in [-180,180]
        double shortest_rot = angle - (currentAngle % 360.0);
        shortest_rot = ((shortest_rot + 180) % 360) - 180;

        // If shortest_rot is too far behind, force forward rotation
        if (shortest_rot < -ANGLE_TOLERANCE && Math.abs(shortest_rot) > BACKWARD_TOLERANCE) {
            shortest_rot += 360.0;
        }

        targetAngle = currentAngle + shortest_rot;
        currentState = State.SEEKING_POSITION;


        currentState = State.SEEKING_POSITION;
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
            this.manualPower = power;
            this.currentState = State.MANUAL_SPIN;
        //}
    }

    // --- State Inquiry ---

    public boolean isAtTarget() {
        return Math.abs(currentAngle - (targetAngle + current_offset)) < ANGLE_TOLERANCE;
    }

    public boolean isHomed() {
        return this.isHomed;
    }


    public double getCurrentAngle() {
        return currentAngle;
    }

    public void update() {
        // --- 1. Cache Hardware Reads ---
        currentAngle = indexMotor.getCurrentPosition() / COUNTS_PER_DEGREE - startingAngle;
        limitSwitchPressed = limitSwitch.isPressed();

        // --- 2. Run State Machine ---
        double power;
        switch (currentState) {
            case IDLE:
                indexerServo1.setPower(0);
                indexerServo2.setPower(0);
                break;

            case HOMING:
                if (limitSwitchPressed) {
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    isHomed = true;
                    targetAngle = 0;
                    current_offset = HOMING_OFFSET;
                    currentState = State.HOLDING_POSITION;
                    isHomed = false;
                } else {
                    indexerServo1.setPower(HOMING_POWER);
                    indexerServo2.setPower(-HOMING_POWER);
                }
                break;

            case MANUAL_SPIN:
                indexerServo1.setPower(manualPower);
                indexerServo2.setPower(-manualPower);
                if (Math.abs(manualPower) < 0.05) {
                    // When driver lets go, find the nearest physical slot and seek to it.
                    double nearestSlotAngle = Math.ceil(currentAngle / 120.0) * 120.0;
                    this.seekToAngle(nearestSlotAngle);
                }
                break;

            case SEEKING_POSITION:
                if (isAtTarget()) {
                    currentState = State.HOLDING_POSITION;
                    // We have arrived. Stop the motor for this one cycle to prevent a "kick".
                    // The next loop will execute the HOLDING_POSITION logic.
                    indexerServo1.setPower(0);
                    indexerServo2.setPower(0);
                } else {
                    // If not at target, continue seeking.
                    angleController.setPID(P, I, D); // Re-apply PID gains from Dashboard
                    power = -angleController.calculate(currentAngle, targetAngle + current_offset);
                    indexerServo1.setPower(power);
                    indexerServo2.setPower(-power);
                }
                break;

                /* --- Old SEEKING_POSITION Implementation ---
                if (isAtTarget()) {
                    currentState = State.HOLDING_POSITION;
                }
                angleController.setPID(P, I, D); // Re-apply PID gains from Dashboard
                power = -angleController.calculate(currentAngle, targetAngle + current_offset);
                indexerServo.setPower(power);
                // Fall-through to HOLDING_POSITION to apply power
                break;
                */

            case HOLDING_POSITION:
                // If a magnet is detected while holding, we use it to correct for encoder drift.
                if (limitSwitchPressed) {
                    // Find the nearest ideal slot angle (0, 120, 240) to our current position.
                    double nearestSlotAngle = Math.round(currentAngle / 120.0) * 120.0;
                    // Snap our PID target to that perfect, calibrated angle.
                    targetAngle = nearestSlotAngle;
                }

                angleController.setPID(P, I, D); // Re-apply PID gains from Dashboard
                power = -angleController.calculate(currentAngle, targetAngle + current_offset);
                indexerServo1.setPower(power);
                indexerServo2.setPower(-power);
                break;
        }

        // --- 3. Telemetry ---
        telemetry.addData("Turnstile State", currentState.name());
//        telemetry.addData("Turnstile Angle", currentAngle);
//        telemetry.addData("Turnstile Target", targetAngle + current_offset);
//        telemetry.addData("Limit Switch Pressed", limitSwitchPressed);
    }
}
