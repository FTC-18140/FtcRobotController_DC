package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import static org.firstinspires.ftc.teamcode.Robot.Turnstile.STARTING_ANGLE_KEY;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

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
    private Telemetry telemetry;

    public static boolean TELEM = false;

    public static double P = 0.0028, I = 0.0, D = 0.00022;
    private State currentState = State.IDLE;

    private boolean isHomed = false;

    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double GEAR_RATIO = (double) 1 / 2;
    private static final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION / 360;

    public enum State {IDLE, HOMING, SEEKING_POSITION, HOLDING_POSITION, LAUNCHING} // Added MANUAL_SPIN
    public static double LAUNCHING_POWER = 0.95;
    public static double HOMING_POWER = 0.065;


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


    public void seekToAngle(double angle)
    {

        angle = ((angle % 360) + 360) % 360; // Make sure angle is within [0, 360]

        // Shortest path shortestRot in [-180,180]
        double shortestRot = angle - (currentAngle % 360.0);
        shortestRot = ((shortestRot + 180) % 360) - 180;

        // If shortestRot is too far behind, force forward rotation
        if (shortestRot < -ANGLE_TOLERANCE && Math.abs(shortestRot) > BACKWARD_TOLERANCE)
        {
            shortestRot += 360.0;
        }
        targetAngle = currentAngle + shortestRot;
        currentState = State.SEEKING_POSITION;
        nearTarget = false;
    }

    public void home()
    {
        currentState = State.HOMING;
        isHomed = false;
    }

    public void launchSlots(int launches) {
        targetAngle = currentAngle - (360);
        currentState = State.LAUNCHING;
    }

    public void update(int count)
    {
        // --- 1. Cache Hardware Reads ---
        currentAngle = indexMotor.getCurrentPosition() / COUNTS_PER_DEGREE - startingAngle - launching_offset;
        limitSwitchPressed = limitSwitch.isPressed();

//        double angleErrorAbs = Math.abs(targetAngle + current_offset - currentAngle);
//        double lowerErrorScalar = (angleErrorAbs * angleErrorAbs) / (ANGLE_TOLERANCE * ANGLE_TOLERANCE);

        double power = 0;
        power = angleController.calculate(currentAngle, targetAngle + current_offset);


        switch (currentState) {
            case IDLE:
                indexerServo1.setPower(0);
                indexerServo2.setPower(0);
                break;
            case HOMING:
                indexerServo1.setPower(HOMING_POWER);
                indexerServo2.setPower(HOMING_POWER);
                isHomed = false;

                if (limitSwitchPressed)
                {
                    indexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    currentAngle = 0;
                    isHomed = true;
                    targetAngle = 0;
//                    current_offset = HOMING_OFFSET;

                    indexerServo1.setPower(power * lowerErrorScalar);
                    indexerServo2.setPower(power * lowerErrorScalar);
                    currentState = State.HOLDING_POSITION;
                }
                break;
            case SEEKING_POSITION:
                indexerServo1.setPower(power);
                indexerServo2.setPower(power);
                if (isAtTarget())
                {
                    // We have arrived. Stop the motor for this one cycle to prevent a "kick".
                    // The next loop will execute the HOLDING_POSITION logic.
                    indexerServo1.setPower(power * lowerErrorScalar);
                    indexerServo2.setPower(power * lowerErrorScalar);
                    currentState = State.HOLDING_POSITION;
                }
                break;
            case LAUNCHING:
                if (currentAngle <= targetAngle) {
                    currentState = State.SEEKING_POSITION;
                    indexerServo1.setPower(0);
                    indexerServo2.setPower(0);
                } else if (3 == numLaunches && currentAngle > (targetAngle - BALL3_ANGLE) && currentAngle <= targetAngle - BALL2_ANGLE) {
                    double fasterPower = Range.clip(-LAUNCHING_POWER * SECOND_BALL_BOOST, -1, 1);
                    indexerServo1.setPower(fasterPower);
                    indexerServo2.setPower(fasterPower);
                } else if (3 == numLaunches && currentAngle <= targetAngle - BALL3_ANGLE) {
                    // If not at target, continue seeking.
                    double fasterPower = Range.clip(-LAUNCHING_POWER * THIRD_BALL_BOOST, -1, 1);
                    indexerServo1.setPower(fasterPower);
                    indexerServo2.setPower(fasterPower);
                } else {

                    // If not at target, continue seeking.
                    indexerServo1.setPower(-LAUNCHING_POWER);
                    indexerServo2.setPower(-LAUNCHING_POWER);
                }
                break;

            case HOLDING_POSITION:
                // If a magnet is detected while holding, we use it to correct for encoder drift.
                if (isAtTarget()) {
                    indexerServo1.setPower(power / 2);
                    indexerServo2.setPower(power / 2);
                } else {
                    indexerServo1.setPower(power);
                    indexerServo2.setPower(power);
                    currentState = Turnstile.State.SEEKING_POSITION;
                }
                break;
        }




    }









}