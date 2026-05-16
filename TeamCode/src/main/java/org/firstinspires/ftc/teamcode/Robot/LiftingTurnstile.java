package org.firstinspires.ftc.teamcode.Robot;
import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.Robot.Turnstile.STARTING_ANGLE_KEY;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_TURNSTILE;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

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

    public static double P = 0.0028, I = 0.0, D = 0.00022;
    public static double stiffP = 0.006, stiffI = 0.0, stiffD = 0.0001;

    private State currentState = State.OFF;

    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION / 360;

    private enum State {OFF, HOMING, CONTROL_TO_ANGLE, LAUNCHING} // Added MANUAL_SPIN

    public static double LAUNCHING_POWER = 0.95;
    public static double CONTROLLING_POWER = 0.7;
    public static double HOMING_POWER = 0.065;
    public static double ANGLE_TOLERANCE = 2.5;// In degrees
    public static double BACKWARD_TOLERANCE = 30;


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

    public void seekToAngle(double toAngle)
    {
        toAngle = ((toAngle % 360) + 360) % 360; // Make sure angle is within [0, 360]

        // Shortest path error in [-180,180]
        double error = toAngle - (currentAngle % 360.0);
        error = ((error + 180) % 360) - 180;

        // If error is too far behind, force forward rotation
        if (error < -ANGLE_TOLERANCE && Math.abs(error) > BACKWARD_TOLERANCE)
        {
            error += 360.0;
        }
        targetAngle = currentAngle + error;
        currentState = State.CONTROL_TO_ANGLE;
    }

    public void home()
    {
        isHomed = false;
        currentState = State.HOMING;
    }

    public void launchSlots(int launches)
    {
        // If launches is 3, this becomes -(120 * 4) = -480 degrees
        double moveAmount = -(120.0 * (launches + 1));
        targetAngle = currentAngle + moveAmount;
        currentState = State.LAUNCHING;
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
                double error = targetAngle - currentAngle;

                // Scheduling Gains
                if (Math.abs(error) < 2*ANGLE_TOLERANCE)
                {
                    if (angleController.getP() != stiffP)
                    {
                        angleController.setPID(stiffP, stiffI, stiffD);
                    }
                }
                else
                {
                    if (angleController.getP() != P)
                    {
                        angleController.setPID(P, I, D); // Snap Gains
                    }
                }
                pwr = angleController.calculate(currentAngle, targetAngle);
                // Deadband to stop hunting in the gear slop
                if (Math.abs(error) < 0.5) pwr = 0;
                // Output Clamp (Safety)
                pwr = Math.max(-CONTROLLING_POWER, Math.min(CONTROLLING_POWER, pwr));
                driveServos(pwr);
                break;
            case LAUNCHING:
                pwr = -LAUNCHING_POWER;
                driveServos(pwr);
                // Switch to PID control when we get close to the target to "brake"
                if (currentAngle <= targetAngle + 60.0)
                {
                    currentState = State.CONTROL_TO_ANGLE;
                }
                break;
        }
        if (DEBUG_TURNSTILE || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Turnstile Angle", currentAngle);
            telemetry.addData("Turnstile Target", targetAngle );
            telemetry.addData("Turnstile Error: ", targetAngle - currentAngle);
            telemetry.addData("Turnstile Power", pwr);
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