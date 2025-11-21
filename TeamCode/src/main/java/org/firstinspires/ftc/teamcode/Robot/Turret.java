package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class Turret {
    // Define the states as an enum
    private enum State {
        HOLDING,
        SEEKING_ANGLE,
        MANUAL_CONTROL
    }

    private State currentState = State.HOLDING; // Initial state

    // Hardware and Utilities
    private CRServo turret;
    private DcMotor turretEnc;
    private PIDController turretAimPID;
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P_TURRET = 0.8, I_TURRET = 0.25, D_TURRET = 0.0;
    public static double MAX_TURRET_POS = 2.0;
    public static double MIN_TURRET_POS = -1.0;
    public static double TURN_SPEED = 208.3; // From original lockOn
    public static double TURRET_DEGREES_PER_SERVO_COMMAND = 0.0048 * ((1.0 / ((double) 40 / 190)) / 360.0);

    // State-specific variables
    private double targetAngle = 0;
    private double manualPower = 0;
    private double currentPosition = 0;

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);

        turret = hwMap.crservo.get("turret");
        turret.setDirection(DcMotor.Direction.REVERSE);
        // The encoder is on the "launcher2" motor in your original file
        try {
            turretEnc = hwMap.get(DcMotor.class, "launcher2");
        } catch (Exception e) {
            telemetry.addData("Motor \"launcher2\" not found", 0);
        }

    }

    // --- High-Level Commands to Change State ---

    public void seekToAngle(double angle) {
        this.targetAngle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);
        this.currentState = State.SEEKING_ANGLE;
    }

    public void setManualPower(double power) {
        this.manualPower = power;
        this.currentState = State.MANUAL_CONTROL;
    }

    public void holdPosition() {
        this.targetAngle = this.currentPosition; // Hold where we are
        this.currentState = State.HOLDING;
    }

    // --- Main Update Method ---

    public void update() {
        updateCurrentPosition(); // Always read the sensor
        turretAimPID.setPID(P_TURRET, I_TURRET, D_TURRET);

        switch (currentState) {
            case HOLDING:
                double holdingPower = -turretAimPID.calculate(currentPosition, targetAngle);
                setHardwarePower(holdingPower);
                break;

            case SEEKING_ANGLE:
                double seekingPower = -turretAimPID.calculate(currentPosition, targetAngle);
                setHardwarePower(seekingPower);
                if (isAtTarget()) {
                    this.currentState = State.HOLDING;
                }
                break;

            case MANUAL_CONTROL:
                setHardwarePower(manualPower);
                if (Math.abs(manualPower) < 0.05) {
                    holdPosition();
                }
                break;
        }

        telemetry.addData("Turret State", currentState.name());
        telemetry.addData("Turret Position", currentPosition);
        telemetry.addData("Turret Target", targetAngle);
    }

    private void setHardwarePower(double power) {
        if (power > 0 && currentPosition <= MIN_TURRET_POS) {
            turret.setPower(0);
        } else if (power < 0 && currentPosition >= MAX_TURRET_POS) {
            turret.setPower(0);
        } else {
            turret.setPower(power);
        }
    }

    private void updateCurrentPosition() {
        this.currentPosition = turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_SERVO_COMMAND;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(this.currentPosition - targetAngle) < 0.03;
    }
}
