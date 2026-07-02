package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchController
{
    TurretController turretController = null;
    public FlywheelController flywheelController = null;

    Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telem, Pose2d pose)
    {
        telemetry = telem;

        turretController = new TurretController();
        turretController.init(hwMap, telemetry, pose);

        flywheelController = new FlywheelController();
        flywheelController.init(hwMap, telemetry);

    }

    public void setTurretStart(double angle){}

    public void update(Pose2d currentPose, PoseVelocity2d currentVelocity)
    {
        turretController.update(currentPose, currentVelocity);
        AimSolution solution = turretController.getSolution();
        flywheelController.update(currentVelocity, solution.distance);
    }

    public boolean isAtTargetRpm()
    {
        return flywheelController.isAtTargetRpm();
    }

    public boolean isAtTarget()
    {
        return turretController.isAtTarget();
    }

    public boolean isManualAiming()
    {
        return turretController.getAimingMode() == TurretController.AimingMode.MANUAL;
    }

    public double getLowerFlywheelRpm()
    {
        return flywheelController.getLowerFlywheelCurrentRPM();
    }

    public double getFlywheelTargetRpm()
    {
        return flywheelController.getLowerFlywheelTargetRPM();
    }

    public double getUpperFlywheelRpm()
    {
        return flywheelController.getUpperFlywheelCurrentRPM();
    }

    public double getUpperFlywheelTargetRpm()
    {
        return flywheelController.getUpperFlywheelTargetRPM();
    }

    public double getTurretAngle() 
    {
        return turretController.getTurretAngle();
    }

    public void setAlliance(ThunderBot2025.Alliance_Color color2025)
    {
        turretController.setAlliance(color2025);
    }

    public double getTotalCurrentDraw()
    {
        return flywheelController.getTotalCurrentDraw() + turretController.getTotalCurrentDraw();
    }

    public void prepShot()
    {
        flywheelController.setMode(FlywheelController.RunMode.DISTANCE);
    }

    void prepShotLow()
    {
        flywheelController.setMode(FlywheelController.RunMode.STATIC);
    }

    void stopFlywheel()
    {
        flywheelController.stop();
    }

    public void autoAim()
    {
        turretController.setAimingMode( TurretController.AimingMode.AUTO);
    }
    public void manualAim()
    {
        turretController.setAimingMode(TurretController.AimingMode.MANUAL);
    }

    public void toggleAim() { turretController.toggleAimingMode();}

    public void holdTurretPosition()
    {
        turretController.holdPosition();
    }

    public void moveAimingTarget(double degrees )
    {
        turretController.manuallyMoveTarget(degrees);
    }
}
