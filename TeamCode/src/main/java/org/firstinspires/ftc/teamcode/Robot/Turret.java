package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
@Config
public class Turret implements DataLoggable {
    // Define the states as an enum
    private enum State {
        HOLDING,
        SEEKING_ANGLE,
        MANUAL_CONTROL
    }

    public State getCurrentState() {
        return currentState;
    }

    private State currentState = State.HOLDING; // Initial state

    // Hardware and Utilities
    private CRServo turret;
    private DcMotor turretEnc;
    private PIDController turretAimPID;
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P_TURRET = 0.017, I_TURRET = 0.001, D_TURRET = 0.001;
    public static double MAX_TURRET_POS = 90;
    public static double MIN_TURRET_POS = -90;
    public static double TURN_SPEED = 208.3; // From original lockOn
    //public static double TURRET_DEGREES_PER_ENCODER_TICK = 0.0048 * ((1.0 / ((double) 40 / 190)) / 360.0);
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 /8192 * 360 * 24/190;


    // State-specific variables
    private double targetAngle = 0;
    private double manualPower = 0;
    private double currentPosition = 0;
    private double offsetAngle = 0;
    private double seekingPower = 0; // Member variable to be accessible for logging

    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);
        try{
            turret = hwMap.crservo.get("turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Motor\"turret\" not found", 0);
        }

        // The encoder is on the "launcher2" motor
        try {
            turretEnc = hwMap.get(DcMotor.class, "launcher2");
            turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Motor \"launcher2\" not found", 0);
        }

    }

    public double getTargetPos(){
        return targetAngle;
    }

    // --- High-Level Commands to Change State ---

    /**
     *
     * @param angle in degrees
     */
    public void seekToAngle(double angle) {
        this.targetAngle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);
        this.currentState = State.SEEKING_ANGLE;
    }

    public void setManualPower(double power) {
        this.manualPower = power;
        this.currentState = State.MANUAL_CONTROL;
    }

    public void setOffsetAngle(double angle) {
        offsetAngle = angle;
    }

    public void holdPosition() {
        this.targetAngle = this.currentPosition; // Hold where we are
        this.currentState = State.HOLDING;
    }

    // --- Main Update Method ---

    public void update() {
        updateCurrentPosition(); // Always read the sensor
        turretAimPID.setPID(P_TURRET, I_TURRET, D_TURRET);
        targetAngle = Range.clip(targetAngle, MIN_TURRET_POS, MAX_TURRET_POS);

        switch (currentState) {
            case HOLDING:
                double holdingPower = -turretAimPID.calculate(currentPosition, targetAngle);
                setHardwarePower(holdingPower);
                break;

            case SEEKING_ANGLE:
                seekingPower = -turretAimPID.calculate(currentPosition, targetAngle);
                setHardwarePower(seekingPower);
                if (isAtTarget()) {
                    this.currentState = State.HOLDING;
                }
                break;

            case MANUAL_CONTROL:
                if((currentPosition < MIN_TURRET_POS && manualPower > 0) || (currentPosition > MAX_TURRET_POS && manualPower < 0)) {
                    this.currentState = State.SEEKING_ANGLE;
                }
                setHardwarePower(manualPower);
                if (Math.abs(manualPower) < 0.05) {
                    holdPosition();
                }
                break;
        }

        telemetry.addData("Turret State", currentState.name());
        telemetry.addData("Turret Position", currentPosition);
        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData("Turret Power", seekingPower);

    }

    private void  setHardwarePower(double power) {
        if (power > 0 && currentPosition <= MIN_TURRET_POS) {
            turret.setPower(0);
        } else if (power < 0 && currentPosition >= MAX_TURRET_POS) {
            turret.setPower(0);
        } else {
            turret.setPower(power);
        }
    }

    private void updateCurrentPosition() {
        this.currentPosition = turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK + offsetAngle;
        telemetry.addData("tc", turretEnc.getCurrentPosition());
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(this.currentPosition - targetAngle) < 0.02*90;
    }

    @Override
    public void logData(DataLogger logger) {
        logger.addField(this.targetAngle);
        logger.addField(this.currentPosition);
        logger.addField(P_TURRET);
        logger.addField(I_TURRET);
        logger.addField(D_TURRET);
        logger.addField(this.seekingPower);
    }
}
