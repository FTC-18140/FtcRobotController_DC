package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * A wrapper for {@link DcMotorEx} that implements a power-caching threshold.
 * <p>
 * This utility reduces traffic on the hardware bus by only sending power commands
 * to the expansion hub when the change in power exceeds a specified delta.
 * This is particularly useful for reducing "jitter" from PID controllers
 * and improving loop times in high-frequency control systems.
 * </p>
 */
public class ThresholdMotor
{
    /**
     * The underlying hardware motor object.
     */
    private final DcMotorEx motor;

    /**
     * The minimum change in power required to trigger a hardware write.
     */
    private final double threshold;

    /**
     * The last power value successfully written to the hardware.
     */
    private double lastPower = 0.0;

    /**
     * Constructs a new ThresholdMotor wrapper.
     *
     * @param motor     The {@link DcMotorEx} hardware instance to wrap.
     * @param threshold The sensitivity of the filter. Changes smaller than this value
     *                  will be ignored unless the target is zero. (e.g., 0.01)
     */
    public ThresholdMotor(DcMotorEx motor, double threshold)
    {
        this.motor = motor;
        this.threshold = threshold;
    }

    /**
     * Commands the motor to the target power, subject to the threshold check.
     * <p>
     * <b>Special Case:</b> If the {@code targetPower} is 0.0 and the motor is
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
            motor.setPower(0);
            lastPower = 0;
            writePerformed = true;
        }
        // Otherwise, only update if the change exceeds the threshold
        else if (Math.abs(targetPower - lastPower) >= threshold)
        {
            motor.setPower(targetPower);
            lastPower = targetPower;
            writePerformed = true;
        }

        return writePerformed;
    }

    /**
     * Returns the underlying hardware motor object.
     * Use this to access methods not proxied by this wrapper, such as
     * {@link DcMotorEx#getVelocity()} or {@link DcMotorEx#getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit)}.
     *
     * @return The original {@link DcMotorEx} instance.
     */
    public DcMotorEx getMotor()
    {
        return motor;
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