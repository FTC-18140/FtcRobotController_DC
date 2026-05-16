package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_FLYWHEEL;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

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
import org.firstinspires.ftc.teamcode.Utilities.ThresholdMotor;

@Config
public class Flywheel
{
    static final double GOBILDA_MOTOR_STALL_CURRENT = 9.2;
    private static final double ENCODER_TICKS_PER_REVOLUTION = 28.0;

    // Define the states as an enum
    private enum State
    {
        IDLE,
        CONTROLLING_SPEED
    }

    private State currentState = State.IDLE; // Initial state

    // Hardware and Utilities
    private DcMotorEx launcher = null;
    private DcMotorEx launcherEnc = null;
    private ThresholdMotor launcherWriter = null;
    private PIDController rpmController = null;
    public static int FILTER_SIZE = 2;
    private MovingAverageFilter rpmFilter = new MovingAverageFilter(FILTER_SIZE);
    private Telemetry telemetry = null;

    // Tunable constants from your original file

    private double P = 0.0, I = 0.0, D = 0.0;
    private double F_MAX = 0.0, F_MIN = 0.0, F_VEL = 0, F_STATIC = 0;

    public static boolean TELEM = false;
    public static double MAX_SHOOTER_RPM = 3000.0;
    public static double MIN_SHOOTER_RPM = 1500.0;
    public static double STATIC_RPM = 1750;
    private static final double SHOOTER_RADIUS = 0.072 / 2.0;
    public static double SPIN_EFFICIENCY = 0.83;
    public double FLYWHEEL_RATIO = 1.0;
    public double FLYWHEEL_GEAR_RATIO = 2.0;


    private double targetRpm = 0;

    public static double RPM_LOWER_BOUND = 25.0;
    public static double RPM_UPPER_BOUND = 20.0;
    public static double POWER_THRESHOLD = 0.01; // Tunable threshold for power updates
    private double currentRpm = 0;
    private double previousRpm = 0;
    private double currentAccel = 0;
    public static double ACCEL_RATE = 0;

    public void init(HardwareMap hwMap, Telemetry telem, String motorName, String encoderName)
    {
        telemetry = telem;
        rpmController = new PIDController(P, I, D);

        launcher = hwMap.get(DcMotorEx.class, motorName);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize the threshold wrapper
        launcherWriter = new ThresholdMotor(launcher, POWER_THRESHOLD);

        if (!motorName.matches(encoderName))
        {
            launcherEnc = hwMap.get(DcMotorEx.class, encoderName);
        }
    }

    void setEncoderReversed()
    {
//        launcherEnc.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setPID(double p, double i, double d)
    {
        P = p;
        I = i;
        D = d;
    }

    void setParameters(double p, double i, double d, double fMin, double fMax, double fVel, double fStatic, double gearRatio, double ratio)
    {
        setPID(p, i, d);
        F_MIN = fMin;
        F_MAX = fMax;
        F_VEL = fVel;
        F_STATIC = fStatic;
        FLYWHEEL_GEAR_RATIO = gearRatio;
        FLYWHEEL_RATIO = ratio;

    }

    // --- High-Level Commands to Change State ---

    /**
     * Commands the flywheel to spin up to a target RPM.
     */
    void setTargetRpm(double rpm)
    {
        targetRpm = Range.clip(rpm * FLYWHEEL_RATIO, MIN_SHOOTER_RPM * FLYWHEEL_RATIO, MAX_SHOOTER_RPM * FLYWHEEL_RATIO);
        currentState = State.CONTROLLING_SPEED;
    }

    double getCurrentRpm()
    {
        return currentRpm;
    }

    double getTargetRpm()
    {
        return targetRpm;
    }

    double getError()
    {
        return targetRpm - currentRpm;
    }

    private double getRPM()
    {
        double tps;
        if (null != launcherEnc)
        {
            tps = launcherEnc.getVelocity();
        }
        else
        {
            tps = launcher.getVelocity();
        }
        return (tps * 60.0) / (ENCODER_TICKS_PER_REVOLUTION * FLYWHEEL_GEAR_RATIO);
    }

    double getCurrentDraw()
    {
        return launcher.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Commands the flywheel to stop.
     */
    public void stop()
    {
        currentState = State.IDLE;
    }

    // --- Main Update Method ---

    /**
     * Call this once per loop. It executes the logic for the current state.
     */
    public void update()
    {
        rpmController.setPID(P, I, D);
        double currentDraw = getCurrentDraw();
        if (currentDraw >= GOBILDA_MOTOR_STALL_CURRENT)
        {
            telemetry.addData("FLYWHEEL STALLED", 0);
        }

        double detectedRpm = rpmFilter.addValue(getRPM());
        if (previousRpm == 0) {previousRpm = currentRpm;}

        if (detectedRpm == previousRpm)
        {
            currentRpm += currentAccel;
        }
        else
        {
            currentRpm = detectedRpm;
        }
        previousRpm = detectedRpm;

        switch (currentState)
        {
            case IDLE:
                setPower(0);
                currentAccel = 0;
                break;
            case CONTROLLING_SPEED:
                // --- Step 1: Calculate the Feedforward value ---

                double scaledPower = Range.scale(targetRpm, MIN_SHOOTER_RPM * FLYWHEEL_RATIO, MAX_SHOOTER_RPM * FLYWHEEL_RATIO, F_MIN, F_MAX);
                double feedforward = Range.clip(scaledPower + F_VEL * targetRpm + F_STATIC, F_MIN, 1.0);

                // --- Step 2: Calculate the PID correction ---
                double pidOutput = rpmController.calculate(currentRpm, targetRpm);
                double clippedPidOutput = Range.clip(pidOutput, -1.0, 1.0);

                // --- Step 3: Combine and Set the Final Power ---
                double finalPower = feedforward + clippedPidOutput;
                currentAccel = clippedPidOutput * ACCEL_RATE;
                setPower(finalPower);

                // --- Telemetry for Debugging ---
                if ( DEBUG_FLYWHEEL || SHOW_DEBUG_ALL)
                {
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

    public void setPower(double power)
    {
        if (launcherWriter != null)
        {
            launcherWriter.setPower(power);
        }
    }

    double calculateWheelRPM(double ballVelocity)
    {
        return (60.0 * ballVelocity) / (2.0 * Math.PI * SHOOTER_RADIUS * SPIN_EFFICIENCY);
    }
}
