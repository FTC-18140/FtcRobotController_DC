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
import org.firstinspires.ftc.teamcode.Utilities.LoopTime;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.ThresholdMotor;

@Config
public class FilteredFlywheel
{
    static final double GOBILDA_MOTOR_STALL_CURRENT = 9.2;
    private static final double ENCODER_TICKS_PER_REVOLUTION = 28.0;
    private double lastRPM = 0;

    // Define the states as an enum
    private enum State
    {
        IDLE,
        CONTROLLING_SPEED,
        POWER
    }

    private State currentState = State.IDLE; // Initial state

    // Hardware and Utilities
    private DcMotorEx launcher = null;
    private DcMotorEx launcherEnc = null;
    private ThresholdMotor launcherWriter = null;
    private PIDController rpmController = null;
    private Telemetry telemetry = null;

    public static double KS = 0.125;
    public static double KV = 0.00034;
//    private double batteryVoltage = 13.0;
//    public static double NOMINAL_VOLTAGE = 13.0;

    private double P = 0.0012, I = 0.0, D = 0.0;

    public static double MAX_SHOOTER_RPM = 3000.0;
    public static double MIN_SHOOTER_RPM = 1500.0;
    public double FLYWHEEL_RATIO = 1.0;
    public double FLYWHEEL_GEAR_RATIO = 2.0;
    private double currentRpm = 0;
    private double targetRpm = 0;
    private double measuredRpm = 0;
    private double predictedRpm = 0;

    private double estimatedDisturbance = 0.0;  // RPM/sec
    public static double POWER_THRESHOLD = 0.01; // Tunable threshold for power updates

    private double filteredRpm = 0;
    private boolean filterInitialized = false;

    private double appliedPower = 0.0;

    // Tunables
//    public static double ALPHA = 0.75;

    // drag coefficient (1/sec)
    // measured the time constant at 0.41 sec.
    public static double K_DRAG = 1.0/0.41;

    public static double K_POWER = 5725.0;

    public static double L1, L2 = 0;
    public static double OBSERVER_POLE = 10;

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


    public void setParameters(double p, double i, double d, double gearRatio)
    {
        setPID(p, i, d);
        FLYWHEEL_GEAR_RATIO = gearRatio;

    }

    /**
     * Commands the flywheel to spin up to a target RPM.
     */
    public void setTargetRpm(double rpm)
    {
//        targetRpm = Range.clip(rpm * FLYWHEEL_RATIO, MIN_SHOOTER_RPM * FLYWHEEL_RATIO, MAX_SHOOTER_RPM * FLYWHEEL_RATIO);
        targetRpm = rpm;
        currentState = State.CONTROLLING_SPEED;
    }

    public double getCurrentDraw()
    {
        return launcher.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Commands the flywheel to stop.
     */
    public void stop()
    {
        currentState = State.IDLE;

        filterInitialized = false;
        filteredRpm = 0;
        currentRpm = 0;
        appliedPower = 0;
    }
    public double getCurrentRpm()
    {
        return currentRpm;
    }

    public double getTargetRpm()
    {
        return targetRpm;
    }

    public double getError()
    {
        return targetRpm - currentRpm;
    }

    // --- Main Update Method ---

    /**
     * Call this once per loop. It executes the logic for the current state.
     */
    public void update()
    {
//        K_POWER = K_DRAG*1864.0/0.75;
        rpmController.setPID(P, I, D);
        double currentDraw = getCurrentDraw();
        if (currentDraw >= GOBILDA_MOTOR_STALL_CURRENT)
        {
            telemetry.addData("FLYWHEEL STALLED", 0);
        }

        lastRPM = measuredRpm;
        measuredRpm = getRPM();  // from Encoder -- this is the sensor reading

        double measuredAccel = (measuredRpm - lastRPM) / LoopTime.LOOP_TIME;

        currentRpm = updateRpmFilter(measuredRpm);

        switch (currentState)
        {
            case IDLE:
                setPower(0);
                if ( DEBUG_FLYWHEEL || SHOW_DEBUG_ALL)
                {
                    telemetry.addLine("Flywheel stopped.");
                }
                break;
            case CONTROLLING_SPEED:
                setPower(calculatePower());
                break;
            case POWER:
                break;
        }
        if ( DEBUG_FLYWHEEL || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Measured RPM", measuredRpm);
            telemetry.addData("Predicted RPM", predictedRpm);
            telemetry.addData("Filtered RPM", currentRpm);
            telemetry.addData("Observer Error", measuredRpm-predictedRpm);
            telemetry.addData("Estimated Disturbance", estimatedDisturbance);
            telemetry.addData("Current Draw", getCurrentDraw());
            telemetry.addData("Requested Power", appliedPower);
        }
    }

    public void setPID(double p, double i, double d)
    {
        P = p;
        I = i;
        D = d;
    }

    public void manual()
    {
        currentState = State.POWER;
    }

    public void setPower(double power)
    {
        appliedPower = power;

        if (launcherWriter != null)
        {
            launcherWriter.setPower(power);
        }
    }

    private double calculatePower()
    {
        double feedforward =  KS + KV*targetRpm;

        double pidOutput = rpmController.calculate(currentRpm, targetRpm);

        telemetry.addData("PID OUTPUT", pidOutput);
        telemetry.addData("FEEDFORWARD", feedforward);

        double clippedPower = feedforward + pidOutput;
        clippedPower = Range.clip(clippedPower, -1.0, 1.0);

        telemetry.addData("CLIPPED", clippedPower);

        return clippedPower;
    }

    private double updateRpmFilter(double measuredRpm)
    {
        double dt = LoopTime.LOOP_TIME;

        if (!filterInitialized)
        {
            filteredRpm = measuredRpm;
            estimatedDisturbance = 0.0;
            filterInitialized = true;
            return filteredRpm;
        }

        //----------------------------------------
        // Observer gains
        //----------------------------------------

        L1 = 2.0 * OBSERVER_POLE - K_DRAG;
        L2 = OBSERVER_POLE * OBSERVER_POLE;

        double L1d = L1 * dt;
        double L2d = L2 * dt;

        //----------------------------------------
        // Prediction
        //----------------------------------------
        double rpmDot =  K_POWER * appliedPower - K_DRAG * filteredRpm + estimatedDisturbance;
        predictedRpm = filteredRpm + rpmDot * dt;

        //----------------------------------------
        // Innovation  -- what is the error of my prediction?
        //----------------------------------------
        double error = measuredRpm - predictedRpm;

        //----------------------------------------
        // Correction -- The L1, L2 terms are like porportional control for the observer... they
        // multiply by the error and are added.
        //----------------------------------------
        filteredRpm = predictedRpm + L1d * error;
        estimatedDisturbance += L2d * error;

        return filteredRpm;
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
}
