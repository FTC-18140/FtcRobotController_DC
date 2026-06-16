package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_TURRET;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretAimSolver
{
    Telemetry telemetry = null;

    private Vector2d targetPos = null;
    private static final Vector2d targetPosBlue = new Vector2d(67.0, 67.0);
    private static final Vector2d targetPosRed = new Vector2d(67.0, -67.0);
    private Pose2d lastOdoPose = null; // Used to calculate delta
    private Pose2d currentOdoPose = null;
    private PoseVelocity2d currentOdoVelocity = null;
    public static double TURRET_OFFSET_X = -0.94488;
    public static double TURRET_OFFSET_Y = -3.04528;
    private double currentTurretAngle = 0.0;
    private AimSolution solution = null;


    public void init(HardwareMap hwMap, Telemetry telem, Pose2d startPose)
    {
        telemetry = telem;

        lastOdoPose = startPose;
        targetPos = targetPosBlue;
    }

    public void update(Pose2d currentOdoPose, PoseVelocity2d currentOdoVelocity, double currentTurretAngle)
    {
        this.currentOdoPose = currentOdoPose;
        this.currentTurretAngle = currentTurretAngle;
        this.currentOdoVelocity = currentOdoVelocity;

        // Handle first loop where lastOdoPose is not yet initialized
        if (null == lastOdoPose)
        {
            lastOdoPose = currentOdoPose;
            // Early exit if this is the very first update and there's no prior pose to calculate delta
            return;
        }

        double angle = getAutoAimAngle();
        double angularVelocity = -currentOdoVelocity.angVel;
        solution = new AimSolution( angle, angularVelocity);

        if (DEBUG_TURRET)
        {
            telemetry.addData("AimSolution Angle", angle);
        }
    }

    public AimSolution getSolution()
    {
        return solution;
    }

    private double getAutoAimAngle()
    {
        double targetTurretAngle;

        // Ensure targetPos is set (e.g., by setAlliance) before calculating.
        // If targetPos is null, we can't calculate a target.
        if (targetPos != null)
        {
            // Vector from Turret offset pos to Goal
            Vector2d trueTargetVector = targetPos.minus(currentOdoPose.position.plus(getTurretOffsetPosInRobotSpace()));

            // Calculate the absolute field-centric angle to the goal (Radians)
            double fieldAngleToGoal = Math.atan2(trueTargetVector.y, trueTargetVector.x);

            // HANDLE IMU WRAPPING:
            // We turn the raw angle into a Rotation2d and subtract our robot heading.
            // This yields the shortest relative distance from robot-front to goal,
            // automatically handling the jump across the +/- 180 degree line.
            double relativeAngleRad = Rotation2d.exp(fieldAngleToGoal).minus(currentOdoPose.heading);

            // Convert result to Degrees for the Turret Subsystem. Negative because turret rotation is reversed.
            targetTurretAngle = -Math.toDegrees(relativeAngleRad);

            // --- NORMALIZATION LOGIC ---
            // Adjust the calculated target angle to be within the continuous range of the turret
            // and closest to its current position, to avoid unnecessary 360-degree rotations.

            while (180.0 < targetTurretAngle - currentTurretAngle)
            {
                targetTurretAngle -= 360.0;
            }
            while (-180.0 >= targetTurretAngle - currentTurretAngle)
            {
                targetTurretAngle += 360.0;
            }
        }
        else
        {
            // If no pose or target is available, effectively hold current position.
            telemetry.addData("Aiming Mode", "IDLE (No Target Found)");
            return currentTurretAngle;
        }
        return targetTurretAngle;
    }

    private Vector2d getTurretOffsetPosInRobotSpace()
    {
        double robotHeading = currentOdoPose.heading.toDouble();
        return new Vector2d(
                TURRET_OFFSET_Y * Math.sin(-robotHeading) + (TURRET_OFFSET_X) * Math.cos(-robotHeading),
                TURRET_OFFSET_Y * Math.cos(-robotHeading) - (TURRET_OFFSET_X) * Math.sin(-robotHeading)
        );
    }

    public void moveTargetForTesting(double degrees)
    {
        if (targetPos != null)
        {
            double rads = Math.toRadians(degrees);
            double cos = Math.cos(rads);
            double sin = Math.sin(rads);

            targetPos = new Vector2d(
                    targetPos.x * cos - targetPos.y * sin,
                    targetPos.x * sin + targetPos.y * cos
            );
        }
    }


}
