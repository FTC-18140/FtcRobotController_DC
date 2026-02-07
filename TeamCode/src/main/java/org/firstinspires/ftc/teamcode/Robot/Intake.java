package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    public static boolean TELEM = true;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try{
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            addTelemetry("DC Motor \"intake\" not found", 0);
        }
    }

    /**
     * sets the intake motor to the preset intake speed
     */
    public void intake(){
        intakeMotor.setPower(INTAKE_SPEED);
        addTelemetry("Intaking", 0);
    }


    public Action intakeStopAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();

                return false;
            }
        };
    }
    public double getIntakePower(){ return intakeMotor.getPower();}

    /**
     * Sets the intake motor to the opposite of the preset intake speed
     */
    public void spit(){
        intakeMotor.setPower(-INTAKE_SPEED);
        addTelemetry("Spitting", 0);
    }

    /**
     * Stops the intake motor
     */
    public void stop() {
        intakeMotor.setPower(0.0);
    }

    public void addTelemetry(String name, Object value) {
        if (TELEM) {
            // this.getClass().getSimpleName() returns "Limelight"
            telemetry.addData("[" + this.getClass().getSimpleName() + "] " + name, value);
        }
    }
    /**     * Overloaded method to format doubles to 2 decimal places automatically
     */
    public void addTelemetry(String name, double value) {
        if (TELEM) {
            telemetry.addData("[" + this.getClass().getSimpleName() + "] " + name, String.format("%.2f", value));
        }
    }

    public void addTelemetry(String line)
    {
        if (TELEM) {
            telemetry.addLine(line);
        }
    }
}
