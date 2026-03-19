package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Flywheel {
    public static final double GOBILDA_MOTOR_STALL_CURRENT = 9.2;

    // Define the states as an enum
    private enum State {
        IDLE,
        SPINNING_UP
    }

    private State currentState = State.IDLE; // Initial state

    // Hardware and Utilities
    private DcMotorEx launcher = null;
    private DcMotorEx launcherEnc = null;
    private PIDController rpmController = null;
    public static int FILTER_SIZE = 2;
    private MovingAverageFilter rpmFilter = new MovingAverageFilter(FILTER_SIZE);
    private Telemetry telemetry = null;

    // Tunable constants from your original file

    private double P = 0.0, I = 0.0, D = 0.0;
    public double F_MAX = 0.0, F_MIN = 0.0;

    public double feedforward = 0.0;

    public static boolean TELEM = false;
    public static double MAX_SHOOTER_RPM = 1900.0;
    public static double MIN_SHOOTER_RPM = 1200.0;
    public static final double SHOOTER_RADIUS = 0.072 / 2.0;
    public static double SPIN_EFFICIENCY = 1.0;
    public double FLYWHEEL_RATIO = (double) 0.9;


    private double targetRpm = (double) 0;

    public static double RPM_LOWER_BOUND = 25.0;
    public static double RPM_UPPER_BOUND = 20.0;

    private double currentRpm = (double) 0;
    private double previousRpm = 0;
    private double currentAccel = 0;
    public static double ACCEL_RATE = 50;
    double scaledPower = (double) 0;
    private double currentDraw = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem, String motorName, String encoderName) {
        this.telemetry = telem;
        rpmController = new PIDController(P, I, D);

        launcher = hwMap.get(DcMotorEx.class, motorName);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (!motorName.matches(encoderName)) {
            launcherEnc = hwMap.get(DcMotorEx.class, encoderName);
        }
    }

    public void setEncoderReversed() {
        launcherEnc.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPID(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
    }

    public void setParameters(double p, double i, double d, double fMin, double fMax, double ratio) {
        setPID(p, i, d);
        this.F_MIN = fMin;
        this.F_MAX = fMax;
        this.FLYWHEEL_RATIO = ratio;
    }

    // --- High-Level Commands to Change State ---

    /**
     * Commands the flywheel to spin up to a target RPM.
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Range.clip(rpm * FLYWHEEL_RATIO, MIN_SHOOTER_RPM * FLYWHEEL_RATIO, MAX_SHOOTER_RPM * FLYWHEEL_RATIO);
        this.currentState = State.SPINNING_UP;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }


    /**
     * Checks if the flywheel is spinning at the target speed within a given tolerance.
     *
     * @return true if the flywheel is at the target RPM, false otherwise.
     */
    public boolean isAtTargetRpm() {
        // We are only "at target" if we are actively trying to spin up.
        if (currentState != State.SPINNING_UP) {
            return false;
        }

        // Check if the current RPM is within a reasonable tolerance (e.g., 50 RPM) of the target.
        // This tolerance can be tuned.
        //final double RPM_TOLERANCE = 20.0;

        return currentRpm - targetRpm < RPM_UPPER_BOUND && currentRpm - targetRpm > -RPM_LOWER_BOUND;
    }

    public double getRPM() {
        double tps;
        if (launcherEnc != null) {
            tps = launcherEnc.getVelocity();
        } else {
            tps = -launcher.getVelocity();
        }
        return (tps * 60.0) / 28.0;
    }

    public double getRpmLowerBound() {
        return RPM_LOWER_BOUND;
    }

    public double getRpmUpperBound() {
        return RPM_UPPER_BOUND;
    }

    public double getCurrentDraw() {
        return launcher.getCurrent(CurrentUnit.AMPS);
    }


    /**
     * Commands the flywheel to stop.
     */
    public void stop() {
        this.currentState = State.IDLE;
    }

    // --- Main Update Method ---

    /**
     * Call this once per loop. It executes the logic for the current state.
     */
    public void update() {

        rpmController.setPID(P, I, D);
        currentDraw = getCurrentDraw();
        if (GOBILDA_MOTOR_STALL_CURRENT <= currentDraw) {
            telemetry.addData("FLYWHEEL STALLED", 0);
        }

        double detectedRpm = rpmFilter.addValue(getRPM());
        if (previousRpm == 0) this.previousRpm = this.currentRpm;

        if (detectedRpm == this.previousRpm) {
            this.currentRpm += currentAccel;
        } else {
            this.currentRpm = detectedRpm;
        }
        this.previousRpm = detectedRpm;


        //telemetry.addData("launcherVel",launcher.getVelocity());

        switch (currentState) {
            case IDLE:
                setPower((double) 0);
                this.currentAccel = 0;
                break;

            case SPINNING_UP:
                // --- Step 1: Calculate the Feedforward value ---

                scaledPower = Range.scale(targetRpm, MIN_SHOOTER_RPM * FLYWHEEL_RATIO, MAX_SHOOTER_RPM * FLYWHEEL_RATIO, F_MIN, F_MAX);
                feedforward = Range.clip(scaledPower, F_MIN, F_MAX);


                // --- Step 2: Calculate the PID correction ---
                double pidOutput = rpmController.calculate(currentRpm, targetRpm);
                double clippedPidOutput = Range.clip(pidOutput, -1.0, 1.0);

                // --- Step 3: Combine and Set the Final Power ---
                double finalPower = feedforward + clippedPidOutput;
                this.currentAccel = clippedPidOutput * ACCEL_RATE;
                setPower(finalPower);

                // --- Telemetry for Debugging ---
                if (TELEM) {
                    telemetry.addData("Target RPM", targetRpm);
                    telemetry.addData("Current RPM", currentRpm);
                    telemetry.addData("RPM Acceleration", currentAccel);
                    telemetry.addData("Feedforward", feedforward);
                    telemetry.addData("PID Output", clippedPidOutput);
                    telemetry.addData("Final Power", finalPower);
                    telemetry.addData("Current Draw", getCurrentDraw());
                }
                break;
        }

    }

    private void setPower(double power) {
        launcher.setPower(power);
    }

    // --- Calculation Methods ---

    public double calculateWheelRPM(double ballVelocity) {
        return (60.0 * ballVelocity) / (2.0 * Math.PI * SHOOTER_RADIUS * SPIN_EFFICIENCY);
    }

}
