package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public static double INTAKE_SPEED = 1.0;
    DcMotor intakeMotor = null;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("DC Motor \"intake\" not found", 0);
        }
    }

    /**
     * sets the intake motor to the preset intake speed
     */
    public void intake(){
        intakeMotor.setPower(INTAKE_SPEED);
    }

    /**
     * Sets the intake motor to the opposite of the preset intake speed
     */
    public void spit(){
        intakeMotor.setPower(-INTAKE_SPEED);
    }

    /**
     * Stops the intake motor
     */
    public void stop() {
        intakeMotor.setPower(0.0);
    }
}
