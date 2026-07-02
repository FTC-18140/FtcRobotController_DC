package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretController
{
    private TurretAimSolver solver;
    private Aimer aimer;

    private boolean enter = true;

    public enum AimingMode
    {AUTO, MANUAL, HOLD}

    private AimingMode aimingMode = AimingMode.AUTO;

    public void init(HardwareMap hwMap, Telemetry telem, Pose2d pose)
    {
        solver = new TurretAimSolver();
        solver.init(hwMap, telem, pose );

        aimer = new Aimer();
        aimer.init(hwMap, telem);
        aimingMode = AimingMode.AUTO;
    }

    public void update(Pose2d pose, PoseVelocity2d vel)
    {
        switch( aimingMode)
        {
            case AUTO:
            case MANUAL:
                solver.update(pose,vel, aimer.getCurrentAngle());
                aimer.setAimSolution(solver.getSolution());
                break;
            case HOLD:
                if ( enter )
                {
                    aimer.holdAtCurrentPosition();
                    enter = false;
                }
//                aimer.holdAtCurrentPosition();
                break;
        }
        aimer.update();
    }

    public AimSolution getSolution()
    {
        return solver.getSolution();
    }

    public boolean isAtTarget()
    {
        return aimer.isAtTarget();
    }

    public double getTurretAngle()
    {
        return aimer.getCurrentAngle();
    }

    public double getTotalCurrentDraw()
    {
       return aimer.getCurrent();
    }

    public void setAimingMode(AimingMode mode)
    {
        enter = true;
        aimingMode = mode;
    }

    public AimingMode getAimingMode()
    {
        return aimingMode;
    }


    public void setAlliance(ThunderBot2025.Alliance_Color color2025)
    {
        solver.setAlliance(color2025);
    }


    public void toggleAimingMode()
    {
        if ( aimingMode != AimingMode.MANUAL)
        {
            setAimingMode(AimingMode.MANUAL);
        }
        else
        {
            setAimingMode(AimingMode.AUTO);
        }
    }

    public void holdPosition()
    {
        setAimingMode(AimingMode.HOLD);
    }
    public void manuallyMoveTarget(double degrees)
    {
        if ( aimingMode == AimingMode.MANUAL)
        {
            solver.moveTargetForTesting(degrees);
        }
    }
}
