package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
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
    private DcMotorEx launcher, launcher2;
    private PIDController rpmController;
    private MovingAverageFilter rpmFilter = new MovingAverageFilter(2);
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P = 0.004, I = 0.0, D = 0.0;
    public static double F_MAX = 0.65, F_MIN = 0.45;
    public static double F_MAX_ADJUST = F_MAX, F_MIN_ADJUST = F_MIN;
    public static double F_STEP = .03;
    public boolean AdjustedFF = false;
    public double feedforward;

    public static double MAX_SHOOTER_RPM = 1010;
    public static double MIN_SHOOTER_RPM = 850;
    public static double SHOOTER_RADIUS = 0.096 / 2.0;
    public static double SPIN_EFFICIENCY = 1.27;

    private double targetRpm = 0;
    private double currentRpm = 0;
    double scaledPower = 0;
    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        rpmController = new PIDController(P, I, D);

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2 = hwMap.get(DcMotorEx.class, "launcher2");
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // --- High-Level Commands to Change State ---

    /** Commands the flywheel to spin up to a target RPM. */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Range.clip(rpm, MIN_SHOOTER_RPM, MAX_SHOOTER_RPM);
        this.currentState = State.SPINNING_UP;
    }


    /**
     * Checks if the flywheel is spinning at the target speed within a given tolerance.
     * @return true if the flywheel is at the target RPM, false otherwise.
     */
    public boolean isAtTargetRpm() {
        // We are only "at target" if we are actively trying to spin up.
        if (currentState != State.SPINNING_UP) {
            return false;
        }

        // Check if the current RPM is within a reasonable tolerance (e.g., 50 RPM) of the target.
        // This tolerance can be tuned.
        final double RPM_TOLERANCE = 50.0;
        return Math.abs(currentRpm - targetRpm) < RPM_TOLERANCE;
    }


    /** Commands the flywheel to stop. */
    public void stop() {
        this.currentState = State.IDLE;
    }

    // --- Main Update Method ---

    /**
     * Call this once per loop. It executes the logic for the current state.
     */
    public void update(double distanceToGoal) {
        rpmController.setPID(P, I, D);
        this.currentRpm = rpmFilter.addValue(-launcher.getVelocity());
        //telemetry.addData("launchervel",launcher.getVelocity());

        switch (currentState) {
            case IDLE:
                setPower(0);
                break;

            case SPINNING_UP:
                // --- Step 1: Calculate the Feedforward value ---
                if (AdjustedFF){
                    scaledPower = Range.scale(distanceToGoal, 60, 130, F_MIN_ADJUST, F_MAX_ADJUST);
                    feedforward = Range.clip(scaledPower, F_MIN_ADJUST, F_MAX_ADJUST);
                } else {
                    scaledPower = Range.scale(distanceToGoal, 60, 130, F_MIN, F_MAX);
                    feedforward = Range.clip(scaledPower, F_MIN, F_MAX);
                }

                // --- Step 2: Calculate the PID correction ---
                double pidOutput = rpmController.calculate(currentRpm, targetRpm);
                double clippedPidOutput = Range.clip(pidOutput, -0.1, 1);

                // --- Step 3: Combine and Set the Final Power ---
                double finalPower = feedforward + clippedPidOutput;
                setPower(finalPower);

                // --- Telemetry for Debugging ---
                telemetry.addData("Target RPM", targetRpm);
                telemetry.addData("Current RPM", currentRpm);
                telemetry.addData("Feedforward", feedforward);
                telemetry.addData("PID Output", clippedPidOutput);
                telemetry.addData("Final Power", finalPower);
                break;
        }
    }
    public void adjustFF(double upDown){
        AdjustedFF = true;
        F_MAX_ADJUST += upDown * F_STEP;
        F_MIN_ADJUST += upDown * F_STEP;
    }
    public void resetFF(){
        AdjustedFF = false;
    }

    private void setPower(double power) {
        launcher.setPower(power);
        launcher2.setPower(power);
    }

    // --- Calculation Methods ---
    public double calculateBallVelocity(double distance, double height, double angleDegrees) {
        double angleRad = Math.toRadians(angleDegrees);
        double g = 9.81;

        double numer = distance * distance * g;
        double denom = (distance * Math.sin(2 * angleRad)) - (2 * height * Math.pow(Math.cos(angleRad), 2));

        denom = Math.max(denom, 0.4);

        telemetry.addData("Denominator: ", denom);
        return Math.sqrt(numer / denom);
    }

    public double calculateWheelRPM(double ballVelocity) {
        return (60.0 * ballVelocity) / (2.0 * Math.PI * SHOOTER_RADIUS * SPIN_EFFICIENCY);
    }
}
