package org.firstinspires.ftc.teamcode.Robot;

public class AimSolution
{
    public double angle;
    public double velocity;
    public double distance;

    public AimSolution(double angle,
                       double angularVelocity)
    {
        this.angle = angle;
        this.velocity = angularVelocity;
        this.distance = 0.0;
    }

    public AimSolution(double angle,
                       double angularVelocity,
                       double distance)
    {
        this.angle = angle;
        this.velocity = angularVelocity;
        this.distance = distance;
    }
}