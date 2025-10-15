package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
<<<<<<< HEAD
=======
import com.qualcomm.robotcore.hardware.DcMotorSimple;
>>>>>>> River_Sandbox
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

<<<<<<< HEAD
    DcMotor intake = null;

    public static double INTAKE_SPEED = 1.0;
=======
    DcMotor intakeMotor = null;
>>>>>>> River_Sandbox

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
<<<<<<< HEAD
            intake = hardwareMap.get(DcMotor.class, "intake");
        } catch (Exception e) {
            telemetry.addData("DC Motor \"intake\" not found", 0);
=======
            intakeMotor = hardwareMap.dcMotor.get("intake");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Motor \"intake\" not found", 0);
>>>>>>> River_Sandbox
        }
    }

    public void intake(){
<<<<<<< HEAD
        intake.setPower(INTAKE_SPEED);
    }
    public void stop() {
        intake.setPower(0.0);
    }
=======
        intakeMotor.setPower(1);
    }
    public void stop(){
        intakeMotor.setPower(0);
    }

>>>>>>> River_Sandbox
}
