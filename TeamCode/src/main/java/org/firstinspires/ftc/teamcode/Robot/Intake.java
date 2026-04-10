package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;

@Config
public class Intake {
    private static final double REV_MOTOR_STALL_CURRENT = 8.5;
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;

    private static double INTAKE_SPEED = 0.7;
    private DcMotor intakeMotor = null;

    private double current_speed = 0;
    private double slow = 1;
    private static double SLOWED_SPEED = 0.4;
    private double currentDraw = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (RuntimeException e) {
            telemetry.addData("DC Motor \"intake\" not found", 0);
        }
    }

    /**
     * Update method for Intake
     */
    public void update() {

        intakeMotor.setPower(current_speed * slow);
        currentDraw = getTotalCurrentDraw();
        if (REV_MOTOR_STALL_CURRENT <= currentDraw) {
            telemetry.addData("INTAKE STALLED", 0);
        }
        telemetry.addData("Intake Current Draw", currentDraw);
    }

    /**
     * sets the intake motor to the preset intake speed
     */
    public void intake() {
        current_speed = INTAKE_SPEED;
        telemetry.addData("Intaking", 0);
    }


    void slow() {
        slow = SLOWED_SPEED;
//        telemetry.addData("Intaking", 0);
    }

    void unslow() {
        slow = 1;
    }


    public Action intakeStopAction() {
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
     *
     * @return the intake power
     */
    double getIntakePower() {
        return intakeMotor.getPower();
    }

    double getTotalCurrentDraw() {
        DcMotorEx intakeMotorEx = (DcMotorEx) intakeMotor;
        return intakeMotorEx.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Sets the intake motor to the opposite of the preset intake speed
     */
    public void spit() {
        current_speed = -INTAKE_SPEED;
        telemetry.addData("Spitting", 0);
    }

    /**
     * Stops the intake motor
     */
    public void stop() {
        current_speed = 0;
    }

    void logData(DataLogger logger) {
        logger.addField(currentDraw);
    }
}
