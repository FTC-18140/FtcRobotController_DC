package org.firstinspires.ftc.teamcode.Robot;
import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.Robot.Turnstile.STARTING_ANGLE_KEY;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_LIFTING_TURNSTILE;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.LoopTime;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class LiftingTurnstile
{
    // --- Hardware & Utilities ---
    private CRServo indexerServo1;
    private CRServo indexerServo2;
    private DcMotorEx indexMotor;
    private TouchSensor limitSwitch;
    private PIDController angleController;
    private double currentAngle = 0;
    private double startingAngle = 0;
    private double targetAngle = 0;
    private boolean limitSwitchPressed = false;
    private boolean isHomed = false;
    private Telemetry telemetry;

    public static double P = 0.0027, I = 0.0, D = 0.00011;
    public static double LAUNCH_D = 0.00015;
    public static double LAUNCH_P = 0.0032;
    private State currentState = State.OFF;

    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION / 360;

    private enum State {OFF, HOMING, CONTROL_TO_ANGLE, LAUNCHING} // Added MANUAL_SPIN

    public static double LAUNCHING_POWER = 0.95;
    public static double CONTROLLING_POWER = 0.35;
    public static double HOMING_POWER = 0.065;
    public static double ANGLE_TOLERANCE = 5;// In degrees
    public static double BACKWARD_TOLERANCE = 30;
    public static double LAUNCH_DECEL_ANGLE = 30;

    public static double INITIAL_CONTROL_POWER = 0.05;
    public static double POWER_RAMP_TIME_CONSTANT = 0.15;

    private double currentControlLimit = INITIAL_CONTROL_POWER;
    ElapsedTime moveTimer = new ElapsedTime();

    public static double MINIMUM_PWR = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        angleController = new PIDController(P, I, D);

        try
        {
            indexerServo1 = hwMap.crservo.get("indexer");
            indexerServo2 = hwMap.crservo.get("indexer2");

            indexMotor = hwMap.get(DcMotorEx.class, MecanumDrive.LEFT_FRONT_MOTOR);
            limitSwitch = hwMap.get(TouchSensor.class, "indexerLimit");

            indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Use our own P
            indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use our own PID
        }
        catch (RuntimeException e)
        {
            telemetry.addData("Turnstile Hardware Not Found", e.getMessage());
        }
        startingAngle = (double) blackboard.getOrDefault(STARTING_ANGLE_KEY, (double) 0);
    }

    public void seekToAngle(double targetAngle)
    {
        targetAngle = ((targetAngle % 360) + 360) % 360; // Make sure angle is within [0, 360]
//        telemetry.addData("Go To This Angle", targetAngle);

        // Shortest path error in [-180,180]
        double error = targetAngle - (currentAngle % 360.0);
        error = ((error + 180) % 360) - 180;

//        telemetry.addData("My Current Angle", currentAngle);
        // If error is too far behind, force forward rotation
        if (error < -ANGLE_TOLERANCE && Math.abs(error) > BACKWARD_TOLERANCE)
        {
            error += 360.0;
        }
//        telemetry.addData("The error -- how much I am off", error);
        this.targetAngle = currentAngle + error;
//        telemetry.addData("My NEW Targer Angle", this.targetAngle);
        currentControlLimit = INITIAL_CONTROL_POWER;
        currentState = State.CONTROL_TO_ANGLE;
        moveTimer.reset();
    }

    public void home()
    {
        isHomed = false;
        currentState = State.HOMING;
    }

    private void launchSlots(int numToLaunch)
    {
        // If numToLaunch is 3, this becomes -(120 * 4) = -480 degrees
        double moveAmount = -(120.0 * numToLaunch);
        targetAngle = currentAngle + moveAmount;
//        currentState = State.LAUNCHING;
        if ( numToLaunch < 3 )
        {
            angleController.setPID(P, I, D);
        }
        else
        {
            angleController.setPID(LAUNCH_P, I, LAUNCH_D);
        }
        currentState = State.CONTROL_TO_ANGLE;
    }

    public void launchAll()
    {
        launchSlots(4);
    }
    public void launchOne()
    {
        launchSlots(1);
    }


    public double getAngleError()
    {
        return Math.abs(targetAngle - currentAngle);
    }
    public boolean isAtTarget()
    {
        return (Math.abs(currentAngle - targetAngle ) < ANGLE_TOLERANCE);
    }

    public boolean isHomed()
    {
        return isHomed;
    }
    public void update()
    {
        // 1. Hardware Read (Keep it at the top)
        currentAngle = indexMotor.getCurrentPosition() / COUNTS_PER_DEGREE + startingAngle;
        limitSwitchPressed = limitSwitch.isPressed();
        double pwr = 0;
        double pidPwr = 0;

        double elapsedMovetime = moveTimer.seconds();
        double alpha = 1.0-Math.exp(-elapsedMovetime/POWER_RAMP_TIME_CONSTANT);

        // Ramp the control power limit up to CONTROLLING_POWER
        currentControlLimit = INITIAL_CONTROL_POWER + alpha*(CONTROLLING_POWER - INITIAL_CONTROL_POWER);

        currentControlLimit = Math.min(currentControlLimit, CONTROLLING_POWER);

        // 2. State Machine
        switch (currentState)
        {
            case OFF:
                pwr = 0;
                stopServos();
                break;
            case HOMING:
                pwr = HOMING_POWER;
                driveServos(pwr);
                if (limitSwitchPressed)
                {
                    finalizeHome();
                    currentState = State.CONTROL_TO_ANGLE;
                }
                break;
            case CONTROL_TO_ANGLE:
//                angleController.setPID(P, I, D); // Gains
                pidPwr = angleController.calculate(currentAngle, targetAngle);
                pwr = pidPwr;
                // Output Clamp (Safety)
                pwr = Range.clip(
                        pwr,
                        -currentControlLimit,
                        currentControlLimit);

                driveServos(pwr);
                break;
            case LAUNCHING:
                pwr = -LAUNCHING_POWER;
                driveServos(pwr);
                // Switch to PID control when we get close to the target to "brake"
                if (currentAngle <= targetAngle + LAUNCH_DECEL_ANGLE)
                {
                    currentControlLimit = INITIAL_CONTROL_POWER;
                    currentState = State.CONTROL_TO_ANGLE;
                }
                break;
        }
        if (DEBUG_LIFTING_TURNSTILE || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Turnstile State", currentState.name());
            telemetry.addData("Turnstile Angle", currentAngle);
            telemetry.addData("Turnstile Target", targetAngle );
            telemetry.addData("Turnstile currentControlLimit", currentControlLimit);
            telemetry.addData("Turnstile Error: ", targetAngle - currentAngle);
            telemetry.addData("Turnstile Power", pwr);
            telemetry.addData("Turnstile PID Pwr Calc", pidPwr);
            telemetry.addData("Limit Switch Pressed", limitSwitchPressed);
        }
    }

    // Helper methods to keep the state machine clean
    private void driveServos(double pwr)
    {
        indexerServo1.setPower(pwr);
        indexerServo2.setPower(pwr);
    }

    private void stopServos()
    {
        driveServos(0);
    }

    private void finalizeHome()
    {
        indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentAngle = 0;
        targetAngle = 0;
        startingAngle = 0;
        isHomed = true;
    }
}