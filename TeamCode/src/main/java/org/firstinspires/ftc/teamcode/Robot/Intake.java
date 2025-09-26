package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    DcMotor intake = null;

    public static double INTAKE_SPEED = 1.0;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            intake = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            telemetry.addData("DC Motor \"intake\" not found", 0);
        }
    }

    public void intake(){
        intake.setPower(INTAKE_SPEED);
    }
    public void stop() {
        intake.setPower(0.0);
    }
}
