package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Flywheel {
    // Define the states as an enum
    private enum State {
        IDLE,
        SPINNING_UP
    }

    private State currentState = State.IDLE; // Initial state

    // Hardware and Utilities
    private DcMotorEx launcher = null;
    private PIDController rpmController = null;
    public static int FILTER_SIZE = 2;
    private MovingAverageFilter rpmFilter = new MovingAverageFilter(FILTER_SIZE);
    private Telemetry telemetry = null;

    // Tunable constants from your original file

    public static double P = 0.0045, I = 0.006, D = 0.00011;
    public static double F_MAX = 0.62, F_MIN = 0.47;

    public double feedforward = 0.0;

    public static boolean TELEM = false;
    public static double MAX_SHOOTER_RPM = 2300.0;
    public static double MIN_SHOOTER_RPM = 1600.0;
    public static final double SHOOTER_RADIUS = 0.072 / 2.0;
    public static double SPIN_EFFICIENCY = 0.586;
    public static double FLYWHEEL_RATIO = (double) 1.0;


    private double targetRpm = (double) 0;

    public static double RPM_LOWER_BOUND = 20.0;
    public static double RPM_UPPER_BOUND = 20.0;

    private double currentRpm = (double) 0;
    double scaledPower = (double) 0;

    public static void flywheel(String[] args) {
        Flywheel launcher = new Flywheel();
        Flywheel activeRoller = new Flywheel();

    }

    public void init(HardwareMap hwMap, Telemetry telem, String motorName) {
        this.telemetry = telem;
        rpmController = new PIDController(P, I, D);

        launcher = hwMap.get(DcMotorEx.class, motorName);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // --- High-Level Commands to Change State ---

    /**
     * Commands the flywheel to spin up to a target RPM.
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Range.clip(rpm, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
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
        double tps = -launcher.getVelocity();
        return (tps * 60.0) / 28.0;
    }

    public double getRpmLowerBound() {
        return RPM_LOWER_BOUND;
    }

    public double getRpmUpperBound() {
        return RPM_UPPER_BOUND;
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
        this.currentRpm = rpmFilter.addValue(getRPM());

        //telemetry.addData("launcherVel",launcher.getVelocity());

        switch (currentState) {
            case IDLE:
                setPower((double) 0);
                break;

            case SPINNING_UP:
                // --- Step 1: Calculate the Feedforward value ---

                scaledPower = Range.scale(targetRpm, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM, F_MIN, F_MAX);
                feedforward = Range.clip(scaledPower, F_MIN, F_MAX);


                // --- Step 2: Calculate the PID correction ---
                double pidOutput = rpmController.calculate(currentRpm, targetRpm);
                double clippedPidOutput = Range.clip(pidOutput, -1.0, 1.0);

                // --- Step 3: Combine and Set the Final Power ---
                double finalPower = feedforward + clippedPidOutput;
                setPower(finalPower);

                // --- Telemetry for Debugging ---
                if (TELEM) {
                    telemetry.addData("Target RPM", targetRpm);
                    telemetry.addData("Current RPM", currentRpm);
                    telemetry.addData("Feedforward", feedforward);
                    telemetry.addData("PID Output", clippedPidOutput);
                    telemetry.addData("Final Power", finalPower);
                }
                break;
        }
    }

    private void setPower(double power) {
        launcher.setPower(power);
    }

    // --- Calculation Methods ---
    public double calculateBallVelocity(double distance, double height, double angleDegrees) {
        double angleRad = Math.toRadians(angleDegrees);
        double g = 9.81;

        double numerator = distance * distance * g;
        double denominator = (distance * Math.sin(2.0 * angleRad)) - (2.0 * height * Math.pow(Math.cos(angleRad), 2.0));

        denominator = Math.max(denominator, 0.4);

        telemetry.addData("Denominator: ", denominator);
        return Math.sqrt(numerator / denominator);
    }

    public double calculateWheelRPM(double ballVelocity) {
        return (60.0 * ballVelocity) / (2.0 * Math.PI * SHOOTER_RADIUS * SPIN_EFFICIENCY);
    }

}
