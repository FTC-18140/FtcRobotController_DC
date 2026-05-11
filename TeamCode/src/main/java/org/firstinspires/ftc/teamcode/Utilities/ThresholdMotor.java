package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ThresholdMotor {
    private final DcMotorEx motor;
    private final double threshold;
    private double lastPower = 0.0;

    /**
     * @param motor The hardware motor to wrap (DcMotorEx for better performance)
     * @param threshold The minimum change required to send a new command
     */
    public ThresholdMotor(DcMotorEx motor, double threshold) {
        this.motor = motor;
        this.threshold = threshold;
    }

    public void setPower(double targetPower) {
        // Special case: Always allow 0 to ensure the robot stops promptly
        if (targetPower == 0 && lastPower != 0) {
            motor.setPower(0);
            lastPower = 0;
        }
        // Otherwise, only update if the change exceeds the threshold
        else if (Math.abs(targetPower - lastPower) >= threshold) {
            motor.setPower(targetPower);
            lastPower = targetPower;
        }
    }

    // Returns the underlying motor for methods like getVelocity() or getCurrent()
    public DcMotorEx getMotor() { return motor; }
    public double getPower() { return lastPower; }
}