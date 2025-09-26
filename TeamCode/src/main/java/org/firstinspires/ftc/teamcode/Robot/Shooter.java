package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor launcher = null;

    public static double SHOOTER_SPEED = 1.0;

    public void init(HardwareMap hwMap, Telemetry telem){
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            launcher = hardwareMap.dcMotor.get("launcher");
        } catch (Exception e) {
            telemetry.addData("DcMotor \"launcher\" not found", 0);
        }
    }

    public void shoot(){
        launcher.setPower(SHOOTER_SPEED);
    }
    public void stop(){
        launcher.setPower(0.0);
    }
}
