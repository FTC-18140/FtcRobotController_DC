package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

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
    private DcMotor turret;
    //private DcMotor turretEnc;
    private PIDController turretAimPID;
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P_TURRET = 0.01, I_TURRET = 0.0005, D_TURRET = 0.001, F_TURRET_MIN = 0.05, F_TURRET_MAX = 0.08;
    public static double MAX_TURRET_POS = 225;
    public static double MIN_TURRET_POS = -90;
    public static double TURRET_ANGLE_TOLERANCE = 3.5;
//    public static double near_limit_resistance_factor_p = 0.0;
//    public static double near_limit_resistance_factor_i = 0.0;
    public static boolean TELEM = true;

    public static double MAX_POWER = 0.8;

    public static double TURN_SPEED = 208.3; // From original lockOn
    //public static double TURRET_DEGREES_PER_ENCODER_TICK = 0.0048 * ((1.0 / ((double) 40 / 190)) / 360.0);
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 /8192 * 360 * 24.24/190.5;


    // State-specific variables
    private double targetAngle = 0;
    private double manualPower = 0;
    private double currentPosition = 0;
    //private double offsetAngle = 0;
    private double seekingPower = 0; // Member variable to be accessible for logging
    public static String STARTING_ANGLE = "TURRET_ENDING_ANGLE_AUTO";
    double startingAngle = (double) blackboard.getOrDefault(STARTING_ANGLE, (double) 0);
    public void init(HardwareMap hwMap, Telemetry telem) {


        currentPosition = startingAngle;
        this.telemetry = telem;
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);
        try{
            turret = hwMap.dcMotor.get("turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Motor\"turret\" not found", 0);
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

//    public void setOffsetAngle(double angle) {
//        offsetAngle = angle;
//    }

    public void holdPosition() {
        this.targetAngle = this.currentPosition; // Hold where we are
        this.currentState = State.HOLDING;
    }

    // --- Main Update Method ---

    public void update() {
        if ( TELEM ) {
            telemetry.addLine(" ------------- TURRET TELEM -------------");
        }

        updateCurrentPosition(); // Always read the sensor

//        double limit_resistance_p = 1 + near_limit_resistance_factor_p * Math.abs(Range.scale(currentPosition, MIN_TURRET_POS, MAX_TURRET_POS, -1, 1));
//        double limit_resistance_i = 1 + near_limit_resistance_factor_p * Math.abs(Range.scale(currentPosition, MIN_TURRET_POS, MAX_TURRET_POS, -1, 1));
//        turretAimPID.setPID(P_TURRET * limit_resistance_p, I_TURRET * limit_resistance_i, D_TURRET);
        turretAimPID.setPID(P_TURRET, I_TURRET, D_TURRET);
        double ff = 0;

//        telemetry.addData("limit resistance: ", limit_resistance_p);
//        telemetry.addData("resisting p: ", P_TURRET * limit_resistance_p);
//        telemetry.addData("resisting i: ", I_TURRET * limit_resistance_i);

        switch (currentState) {
            case HOLDING:
                seekingPower = turretAimPID.calculate(currentPosition, targetAngle);
                ff = Range.clip(Range.scale(currentPosition, -90, -30, F_TURRET_MAX, F_TURRET_MIN), F_TURRET_MIN, F_TURRET_MAX);
                ff *= ((seekingPower >= 0) ? 1 : -1);

                setHardwarePower(seekingPower + ff);
                break;

            case SEEKING_ANGLE:
                seekingPower = turretAimPID.calculate(currentPosition, targetAngle);
                ff = Range.clip(Range.scale(currentPosition, -90, -30, F_TURRET_MAX, F_TURRET_MIN), F_TURRET_MIN, F_TURRET_MAX);
                ff *= ((seekingPower >= 0) ? 1 : -1);

                setHardwarePower(seekingPower + ff);
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

        if ( TELEM ) {
            telemetry.addData("Turret Starting Angle: ", startingAngle);
            telemetry.addData("Turret Position: ", currentPosition);
            telemetry.addData("Turret Target: ", targetAngle);
            telemetry.addData("Turret Power: ", seekingPower + ff);
            telemetry.addData("Turret State: ", currentState);
        }

    }

    private void  setHardwarePower(double power) {
        if (power < 0 && currentPosition + power*45 <= MIN_TURRET_POS) {
            telemetry.addData("Turret position power override value: ", currentPosition + power*45);
            telemetry.addData("Turret Power sent to hardware: ", 0);
            turret.setPower(0);
        } else if (power > 0 && currentPosition + power*45 >= MAX_TURRET_POS) {
            telemetry.addData("Turret position power override value: ", currentPosition + power*45);
            telemetry.addData("Turret Power sent to hardware: ", 0);
            turret.setPower(0);
        } else {
            power = Range.clip(power, -MAX_POWER, MAX_POWER);
            telemetry.addData("Turret Power sent to hardware", power);
            turret.setPower(power);
        }
    }

    private void updateCurrentPosition() {
        this.currentPosition = turret.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK + startingAngle;
        //this.currentPosition = turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK - offsetAngle;
        //telemetry.addData("tc", turretEnc.getCurrentPosition());
    }
    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(this.currentPosition - targetAngle) < TURRET_ANGLE_TOLERANCE;
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
