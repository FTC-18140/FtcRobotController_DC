package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretController
{
    private TurretAimSolver solver;
    private Aimer aimer;

    public enum AimingMode
    {AUTO, MANUAL}

    private AimingMode aimingMode = AimingMode.AUTO;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        solver = new TurretAimSolver();
        solver.init(hwMap, telem, new Pose2d(0.0, 0.0, 0.0));

        aimer = new Aimer();
        aimer.init(hwMap, telem);
        aimingMode = AimingMode.AUTO;
    }

    public void update(Pose2d pose, PoseVelocity2d vel)
    {
        solver.update(pose,vel, aimer.getCurrentAngle());
        if ( aimingMode == AimingMode.AUTO)
        {
            aimer.setAimSolution(solver.getSolution());
        }
        aimer.update();
    }

    public void testMoveTarget(double degrees)
    {
        solver.moveTargetForTesting(degrees);
    }
}
