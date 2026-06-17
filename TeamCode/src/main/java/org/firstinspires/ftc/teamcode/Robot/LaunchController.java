package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchController
{
    TurretController turretController = null;
    FlywheelController flywheelController = null;

    Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;

        turretController = new TurretController();
        turretController.init(hwMap, telemetry);

        flywheelController = new FlywheelController();
        flywheelController.init(hwMap, telemetry);

    }

    public void setTurretStart(double angle){}

    public void update()
    {
        turretController.update();
        flywheelController.update();
    }

}
