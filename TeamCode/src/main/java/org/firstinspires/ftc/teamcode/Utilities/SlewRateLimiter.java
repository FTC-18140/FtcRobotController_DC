package org.firstinspires.ftc.teamcode.Utilities;


public class SlewRateLimiter
{
    private double value = 0.0;
    private double maxDelta;

    public SlewRateLimiter(double maxDelta)
    {
        this.maxDelta = maxDelta;
    }

    public double update(double target)
    {
        double delta = target - value;

        if (Math.abs(delta) > maxDelta)
        {
            value += Math.signum(delta) * maxDelta;
        }
        else
        {
            value = target;
        }

        return value;
    }
}