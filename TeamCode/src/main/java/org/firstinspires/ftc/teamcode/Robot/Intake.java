package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    DcMotor intakeMotor = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            intakeMotor = hardwareMap.dcMotor.get("intake");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Motor \"intake\" not found", 0);
        }
    }

    public void intake(){
        intakeMotor.setPower(1);
    }
    public void stop(){
        intakeMotor.setPower(0);
    }

}
