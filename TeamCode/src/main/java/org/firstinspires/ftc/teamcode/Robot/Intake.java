package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public static double INTAKE_SPEED = 1.0;
    DcMotor intakeMotor = null;

    private double current_speed = 0;
    private double slow = 1;
    public static double SLOWED_SPEED = 0.6;

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
     * Update method for Intake
     */
    public void update(){
        intakeMotor.setPower(current_speed * slow);
    }

    /**
     * sets the intake motor to the preset intake speed
     */
    public void intake(){
        current_speed = INTAKE_SPEED;
        telemetry.addData("Intaking", 0);
    }


    public void slow(){
        slow = SLOWED_SPEED;
//        telemetry.addData("Intaking", 0);
    }
    public void unslow(){
        slow = 1;
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

    /**
     * Used to expose the intake power value to other classes
     * @return the intake power
     */
    public double getIntakePower(){ return intakeMotor.getPower();}

    /**
     * Sets the intake motor to the opposite of the preset intake speed
     */
    public void spit(){
        current_speed = -INTAKE_SPEED;
        telemetry.addData("Spitting", 0);
    }

    /**
     * Stops the intake motor
     */
    public void stop() {
        current_speed = 0;
    }
}
