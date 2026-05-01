package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;

@Config
public class Intake {
    private static final double REV_MOTOR_STALL_CURRENT = 8.5;
    private Telemetry telemetry = null;
    private HardwareMap hardwareMap = null;

    public static double INTAKE_MOTOR_POWER = 0.85;
    public static double INTAKE_SERVO_POWER = 1.0;

    private DcMotor intakeMotor = null;
    public CRServo intakeServo = null;

    private double intakeMotorPower = 0;
    private double intakeServoPower = 0;

    private double slow = 1;
    private double reversed = 1;
    private static double SLOWED_POWER_FACTOR = 0.85;
    private double currentDraw = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem) {
        hardwareMap = hwMap;
        telemetry = telem;

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        } catch (RuntimeException e) {
            telemetry.addData("DC Motor \"intake\" not found", 0);
        }
        try {
            intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        } catch (RuntimeException e) {
            telemetry.addData("CR Servo \"intake\" not found", 0);
        }
    }

    /**
     * Update method for Intake
     */
    public void update(boolean indexing) {
        if (indexing) {
            intakeServo.setPower(INTAKE_SERVO_POWER);
        } else {
            intakeServo.setPower(intakeServoPower);
        }
        if(reversed == -1) {
            intakeMotor.setPower(INTAKE_MOTOR_POWER * slow * reversed);
        } else {
            intakeMotor.setPower(intakeMotorPower * slow * reversed);
        }
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
        motorIntake();
        servoIntake();
        telemetry.addData("Intaking", 0);
    }

    public void servoIntake() {
        intakeServoPower = INTAKE_SERVO_POWER;
    }

    public void motorIntake() {
        intakeMotorPower = INTAKE_MOTOR_POWER;
        reversed = 1;
    }

    void slow() {
        slow = SLOWED_POWER_FACTOR;
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
    double getIntakeMotorPower() {
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
        motorSpit();
        servoStop();
        telemetry.addData("Spitting", 0);
    }
    public void unSpit() {
        reversed = 1;
    }

    public void stop() {
        motorStop();
        servoStop();
    }

    public void motorStop() {
        intakeMotorPower = 0;
    }

    public void servoStop() {
        intakeServoPower = 0;
    }

    public void motorSpit() {
        reversed = -1;
    }

    public void servoSpit() {
        intakeServoPower = -INTAKE_SERVO_POWER;
    }

    public void DEBUG_intakeMotor(){
        intakeMotor.setPower(0.5);
    }


    void logData(DataLogger logger) {
        logger.addField(currentDraw);
    }
}
