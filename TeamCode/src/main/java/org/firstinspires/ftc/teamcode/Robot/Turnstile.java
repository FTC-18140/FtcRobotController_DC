package org.firstinspires.ftc.teamcode.Robot;

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
    private CRServo indexerServo;
    private DcMotorEx indexMotor;
    private TouchSensor limitSwitch;
    private PIDController angleController;
    private Telemetry telemetry;


    // --- Tunable Constants via FTC Dashboard ---
    public static double P = 0.0016, I = 0.01, D = 0.0001;
    public static double HOMING_POWER = -0.05;
    public static double ANGLE_TOLERANCE = 2.0; // In degrees

    // --- Non-tunable Constants ---
    private static final double COUNTS_PER_REVOLUTION = 288;
    private static final double GEAR_RATIO = 2.0;
    private static final double COUNTS_PER_DEGREE = (COUNTS_PER_REVOLUTION * GEAR_RATIO)/(7.5*Math.PI);

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
            indexerServo = hwMap.crservo.get("indexer");
            indexMotor = hwMap.get(DcMotorEx.class, "indexMotor");
            limitSwitch = hwMap.get(TouchSensor.class, "indexerLimit");

            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Use our own P
            indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use our own PID
        } catch (Exception e) {
            telemetry.addData("Turnstile Hardware Not Found", e.getMessage());
        }
    }

    // --- High-Level Commands ---

    public void home() {
        if (!isHomed) {
            currentState = State.HOMING;
        }
    }

    public void seekToAngle(double angle) {
        // Refactored to have a single exit point
        //if (isHomed) {
            this.targetAngle = angle;
            this.currentState = State.SEEKING_POSITION;
        //}
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
        return Math.abs(currentAngle - targetAngle) < ANGLE_TOLERANCE;
    }

    public boolean isHomed() {
        return this.isHomed;
    }

    public void update() {
        // --- 1. Cache Hardware Reads ---
        currentAngle = indexMotor.getCurrentPosition() / COUNTS_PER_DEGREE;
        limitSwitchPressed = limitSwitch.isPressed();

        // --- 2. Run State Machine ---
        double power;
        switch (currentState) {
            case IDLE:
                indexerServo.setPower(0);
                break;

            case HOMING:
                if (limitSwitchPressed) {
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    isHomed = true;
                    targetAngle = 0;
                    currentState = State.HOLDING_POSITION;
                } else {
                    indexerServo.setPower(HOMING_POWER);
                }
                break;

            case MANUAL_SPIN:
                indexerServo.setPower(manualPower);
                if (Math.abs(manualPower) < 0.05) {
                    // When driver lets go, find the nearest physical slot and seek to it.
                    double nearestSlotAngle = Math.round(currentAngle / 120.0) * 120.0;
                    this.seekToAngle(nearestSlotAngle);
                }
                break;

            case SEEKING_POSITION:
                if (isAtTarget()) {
                    currentState = State.HOLDING_POSITION;
                }
                angleController.setPID(P, I, D); // Re-apply PID gains from Dashboard
                power = -angleController.calculate(currentAngle, targetAngle);
                indexerServo.setPower(power);
                // Fall-through to HOLDING_POSITION to apply power
                break;

            case HOLDING_POSITION:
                // If a magnet is detected while holding, we use it to correct for encoder drift.
                if (limitSwitchPressed) {
                    // Find the nearest ideal slot angle (0, 120, 240) to our current position.
                    double nearestSlotAngle = Math.round(currentAngle / 120.0) * 120.0;
                    // Snap our PID target to that perfect, calibrated angle.
                    targetAngle = nearestSlotAngle;
                }

                angleController.setPID(P, I, D); // Re-apply PID gains from Dashboard
                power = -angleController.calculate(currentAngle, targetAngle);
                indexerServo.setPower(power);
                break;
        }

        // --- 3. Telemetry ---
        telemetry.addData("Turnstile State", currentState.name());
        telemetry.addData("Turnstile Angle", currentAngle);
        telemetry.addData("Turnstile Target", targetAngle);
        telemetry.addData("Limit Switch Pressed", limitSwitchPressed);
    }
}
