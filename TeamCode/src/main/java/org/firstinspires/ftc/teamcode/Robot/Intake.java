package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_INTAKE;
import static org.firstinspires.ftc.teamcode.TelemetryConfig.SHOW_DEBUG_ALL;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;

@Config
public class Intake
{
    // Constants
    public static double INTAKE_POWER = 0.85;
    public static double SERVO_POWER = 1.0;
    public static double SLOW_INTAKE_POWER = 0.65;
    private static final double STALL_CURRENT_LIMIT = 8.5;

    // Hardware
    private DcMotorEx intakeMotor;
    private CRServo intakeServo;
    private Telemetry telemetry;
    private BeamBreakSystem beamBreak = new BeamBreakSystem();
    private SlewRateLimiter intakeRamp = new SlewRateLimiter(0.08);

    private ElapsedTime jamTimer = new ElapsedTime();
    private boolean entry = true;

    public Action intakeStopAction()
    {
        return packet ->
        {
            stop();
            return false;
        };
    }

    public void DEBUG_intakeMotor()
    {

    }

    // State
    public enum State
    {
        INTAKING,
        SPITTING,
        JAMMED,
        STOPPED
    }

    private State currentState = State.STOPPED;
    private boolean runSlow = false;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        this.telemetry = telem;

        try
        {
            intakeMotor = hwMap.get(DcMotorEx.class, "intake");
            intakeServo = hwMap.get(CRServo.class, "intakeServo");
        }
        catch (Exception e)
        {
            telemetry.addData("Error", "Intake hardware not found: " + e.getMessage());
        }
        beamBreak.init(hwMap, telem);

    }

    /**
     * Returns the current draw of the intake motor.
     * This satisfies the requirement for getTotalMotorCurrentDraw() in your robot classes.
     */
    public double getTotalCurrentDraw()
    {
        if (intakeMotor != null)
        {
            return intakeMotor.getCurrent(CurrentUnit.AMPS);
        }
        return 0.0;
    }

    /**
     * Main update loop. Call this in every hardware loop.
     *
     * @param indexing If true, forces the servo to run (used for transfer logic)
     */
    public void update(boolean indexing)
    {
        double powerMultiplier = runSlow ? SLOW_INTAKE_POWER : 1.0;
        double currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);

        beamBreak.update();
        double motorPower = 0.0;
        switch (currentState)
        {
            case INTAKING:
                if ( currentDraw > STALL_CURRENT_LIMIT && Math.abs(intakeMotor.getVelocity()) < 50 )
                {
                    changeState(State.JAMMED);
                }
                else
                {
                    motorPower = runSlow ? SLOW_INTAKE_POWER : INTAKE_POWER;
    //                motorPower = intakeRamp.update(motorPower);
    //                intakeMotor.setPower(motorPower);
    //                intakeMotor.setPower(runSlow ? SLOW_INTAKE_POWER : INTAKE_POWER);
                    intakeServo.setPower(SERVO_POWER);
                }
                break;
            case SPITTING:
                 motorPower = -SLOW_INTAKE_POWER;
//                intakeMotor.setPower(-SLOW_INTAKE_POWER);
//                intakeServo.setPower(-SERVO_POWER);
                if (!beamBreak.ballInIntake())
                {
                    changeState(State.STOPPED);
                }
                break;
            case JAMMED:
                if (entry)
                {
                    jamTimer.reset();
                    entry = false;
                }
                else if ( jamTimer.seconds() > 0.5)
                {
                    changeState(State.INTAKING);
                }
                motorPower = -0.2;
                break;
            case STOPPED:
//                intakeMotor.setPower(0);
                // If indexing is true, run servo even if motor is stopped
                intakeServo.setPower(indexing ? SERVO_POWER : 0);
                break;
        }
        motorPower = intakeRamp.update(motorPower);
        intakeMotor.setPower(motorPower);

        if (currentDraw >= STALL_CURRENT_LIMIT)
        {
            telemetry.addLine("!!! INTAKE STALLED !!!");
        }

        if (DEBUG_INTAKE || SHOW_DEBUG_ALL)
        {
            telemetry.addData("Intake State", currentState);
            telemetry.addData("Intake Current", currentDraw);
        }
    }

    // --- State Control Methods ---

    public void intake()
    {
        currentState = State.INTAKING;
    }

    public void spit()
    {
        currentState = State.SPITTING;
    }

    public void stop()
    {
        currentState = State.STOPPED;
    }


    // --- RoadRunner Actions ---

    public Action actionIntake()
    {
        return packet ->
        {
            intake();
            return false; // Run once
        };
    }

    public Action actionStop()
    {
        return packet ->
        {
            stop();
            return false;
        };
    }

    public Action actionSpit(double seconds)
    {
        return new Action()
        {
            private double startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if (startTime < 0)
                {
                    startTime = System.currentTimeMillis() / 1000.0;
                }

                double now = System.currentTimeMillis() / 1000.0;
                if (now - startTime < seconds)
                {
                    spit();
                    return true; // Keep running
                }
                else
                {
                    stop();
                    return false; // Finished
                }
            }
        };
    }

    private void changeState( State newState )
    {
        entry = true;
        currentState = newState;
    }
    // Add these to the refactored Intake.java so your old code still compiles
    public void setSlowSpeed()
    {
        runSlow = true;
    }

    public void setNormalSpeed()
    {
        runSlow = false;
    }

    public void unSpit()
    {
        intake();
    } // Or stop(), depending on what you want

    public void motorIntake()
    {
        intake();
    }

    public void motorSpit()
    {
        spit();
    }

    public void motorStop()
    {
        stop();
    }
}