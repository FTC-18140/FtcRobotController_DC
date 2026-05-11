package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * A wrapper for {@link CRServo} that implements a power-caching threshold.
 * <p>
 * This utility reduces traffic on the hardware bus by only sending power commands
 * to the expansion hub when the change in power exceeds a specified delta.
 * This is particularly useful for reducing "jitter" from PID controllers
 * and improving loop times in high-frequency control systems.
 * </p>
 */
public class ThresholdServo
{
    /**
     * The underlying hardware servo object.
     */
    private final CRServo servo;

    /**
     * The minimum change in power required to trigger a hardware write.
     */
    private double threshold;

    /**
     * The last power value successfully written to the hardware.
     */
    private double lastPower = 0.0;

    /**
     * Constructs a new ThresholdServo wrapper.
     *
     * @param servo     The {@link CRServo} hardware instance to wrap.
     * @param threshold The sensitivity of the filter. Changes smaller than this value
     *                  will be ignored unless the target is zero. (e.g., 0.01)
     */
    public ThresholdServo(CRServo servo, double threshold)
    {
        this.servo = servo;
        this.threshold = threshold;
    }

    /**
     * Constructs a new ThresholdServo wrapper.
     *
     * @param servo     The {@link CRServo} hardware instance to wrap.
     */
    public ThresholdServo(CRServo servo)
    {
        this.servo = servo;
        this.threshold = 0.001;
    }


    /**
     * Commands the servo to the target power, subject to the threshold check.
     * <p>
     * <b>Special Case:</b> If the {@code targetPower} is 0.0 and the servo is
     * currently moving, the command is always sent immediately to ensure
     * the robot stops promptly for safety.
     * </p>
     *
     * @param targetPower The desired power level, typically between -1.0 and 1.0.
     * @return {@code true} if a command was actually sent to the hardware;
     * {@code false} if the update was skipped due to the threshold.
     */
    public boolean setPower(double targetPower)
    {
        boolean writePerformed = false;

        // Special case: Always allow 0 to ensure the robot stops promptly
        if (targetPower == 0 && lastPower != 0)
        {
            setPowerRaw(0);
            writePerformed = true;
        }
        // Otherwise, only update if the change exceeds the threshold
        else if (Math.abs(targetPower - lastPower) >= threshold)
        {
            setPowerRaw(targetPower);
            writePerformed = true;
        }
        return writePerformed;
    }

    /**
     * Commands the servo to the target power immediately, bypassing the threshold check.
     * <p>
     * Use this method for critical movements where exact power matching is required
     * regardless of hardware bus traffic. Calling this method updates the internal
     * cache, which will affect the logic of subsequent {@link #setPower(double)} calls.
     * </p>
     *
     * @param targetPower The desired power level, typically between -1.0 and 1.0.
     */
    public void setPowerRaw(double targetPower)
    {
        servo.setPower(targetPower);
        lastPower = targetPower;
    }

    /**
     * Gets the current caching tolerance (threshold).
     *
     * @return The minimum change in power required to trigger a hardware write.
     */
    public double getThreshold()
    {
        return threshold;
    }

    /**
     * Sets the caching tolerance (threshold) for power updates.
     * <p>
     * Increasing this value reduces hardware bus traffic but may decrease
     * control precision. Decreasing it improves precision at the cost of
     * higher bus utilization.
     * </p>
     *
     * @param threshold The new sensitivity delta. Should be a non-negative value.
     */
    public void setThreshold(double threshold)
    {
        this.threshold = Math.max(0, threshold);
    }

    /**
     * Returns the underlying hardware servo object.
     * Use this to access methods not proxied by this wrapper, such as
     * {@link CRServo#getDirection()}.
     *
     * @return The original {@link CRServo} instance.
     */
    public CRServo getServo()
    {
        return servo;
    }

    /**
     * Gets the last power value that was successfully written to the hardware.
     * Note: This may differ from the last value passed to {@link #setPower(double)}
     * if that value did not meet the threshold.
     *
     * @return The current cached power.
     */
    public double getPower()
    {
        return lastPower;
    }
}
