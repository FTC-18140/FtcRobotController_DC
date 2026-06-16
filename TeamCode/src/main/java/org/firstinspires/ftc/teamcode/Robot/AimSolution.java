package org.firstinspires.ftc.teamcode.Robot;

public class AimSolution
{
    public double angle;
    public double velocity;
    public double acceleration;

    public AimSolution(double angle,
                       double angularVelocity)
    {
        this.angle = angle;
        this.velocity = angularVelocity;
        this.acceleration = 0.0;
    }

    public AimSolution(double angle,
                       double angularVelocity, double acceleration)
    {
        this.angle = angle;
        this.velocity = angularVelocity;
        this.acceleration = acceleration;
    }
}