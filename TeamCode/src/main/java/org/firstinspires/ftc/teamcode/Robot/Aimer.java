package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_TURRET;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Auto.AutoRedDepot_12;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.ThresholdMotor;

public class Aimer
{

    private DcMotorEx turret = null;
    private ThresholdMotor turretWriter = null;
    private DcMotorEx turretEnc = null;
    private PIDController turretAimPID = null;
    private Telemetry telemetry = null;
    private double currentAngle = 0.0;
    private double startingAngle = 0.0;
    private double targetAngle = 0.0;
    private Aimer.State currentState = Aimer.State.HOLDING;
    private boolean enter = true;

    private double manualPower = 0.0;

    public static double P = 0.0136, I = 0.0, D = 0.001645;
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 / 8192.0 * 360.0 * 16 / 100;
    public static double TURRET_POWER_THRESHOLD = 0.005;
    public static double MAX_POWER = 0.6;

    public static double TURRET_ANGLE_TOLERANCE = 3.5;

    public enum State
    {STOP, HOLDING, SEEKING_ANGLE, MANUAL_CONTROL}

    public static String STARTING_ANGLE_KEY = AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        startingAngle = (double) OpMode.blackboard.getOrDefault(STARTING_ANGLE_KEY, (double) 0);
        turretAimPID = new PIDController(P, I, D);

        try
        {
            turret = hwMap.get(DcMotorEx.class, "turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretWriter = new ThresholdMotor(turret, TURRET_POWER_THRESHOLD);

            turretEnc = hwMap.get(DcMotorEx.class, "rightBack");
            turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (RuntimeException e)
        {
            telemetry.addData("Turret hardware not found", e.getMessage());
        }
    }

    public void update()
    {
        updateCurrentPosition();
        double pidPower = turretAimPID.calculate(currentAngle, targetAngle);

        switch (currentState)
        {
            case HOLDING:
            case SEEKING_ANGLE:
                if (enter)
                {
                    enter = false;
                }
                if (isAtTarget())
                {
                    setHardwarePower(pidPower);
                    currentState = State.HOLDING;
                }
                else
                {
                    setHardwarePower(pidPower);
                    currentState = State.SEEKING_ANGLE;
                }
                break;
            case MANUAL_CONTROL:
                if (enter)
                {
                    enter = false;
                }
                setHardwarePower(manualPower);
                if (Math.abs(manualPower) < 0.01)
                {
                    currentState = State.HOLDING;
                }
                break;
            case STOP:
                if (enter)
                {
                    enter = false;
                }
                setHardwarePower(0);
                break;
        }
        if (DEBUG_TURRET || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Turret Position", "%.2f", currentAngle);
            telemetry.addData("Turret Target", "%.2f", targetAngle);
            telemetry.addData("Turret PID Power", "%.3f", pidPower);
            telemetry.addData("Turret State", currentState);
        }
    }

    public boolean isAtTarget()
    {
        return Math.abs(currentAngle - targetAngle) < TURRET_ANGLE_TOLERANCE;
    }

    private void updateCurrentPosition()
    {
        double rawDegrees = -turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK;
        currentAngle = rawDegrees + startingAngle;
    }

    private void setHardwarePower(double power)
    {
        if (turretWriter != null) {turretWriter.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));}
    }

    public void changeState( State newState )
    {
        enter = true;
        currentState = newState;
    }

}
