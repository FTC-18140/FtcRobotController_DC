package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    CRServo intake = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            intake = hardwareMap.get(CRServo.class, "intake");
        } catch (Exception e) {
            telemetry.addData("Continuous Servo \"intake\" not found", 0);
        }
    }
}
