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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake
{
    // Constants
    public static double MOTOR_POWER = 0.85;
    public static double SERVO_POWER = 1.0;
    public static double SLOW_FACTOR = 0.85;
    private static final double STALL_CURRENT_LIMIT = 8.5;

    // Hardware
    private DcMotorEx intakeMotor;
    private CRServo intakeServo;
    private Telemetry telemetry;

    public Action intakeStopAction() {
        return packet -> {
            stop();
            return false;
        };
    }

    public void DEBUG_intakeMotor() {

    }

    // State
    public enum State
    {
        INTAKING,
        SPITTING,
        STOPPED
    }

    private State currentState = State.STOPPED;
    private boolean isSlowed = false;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;

        try
        {
            intakeMotor = hwMap.get(DcMotorEx.class, "intake");
            intakeServo = hwMap.get(CRServo.class, "intakeServo");
        }
        catch (Exception e)
        {
            telemetry.addData("Error", "Intake hardware not found: " + e.getMessage());
        }
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
        double powerMultiplier = isSlowed ? SLOW_FACTOR : 1.0;
        double currentDraw = intakeMotor.getCurrent(CurrentUnit.AMPS);

        switch (currentState)
        {
            case INTAKING:
                intakeMotor.setPower(MOTOR_POWER * powerMultiplier);
                intakeServo.setPower(SERVO_POWER);
                break;
            case SPITTING:
                intakeMotor.setPower(-MOTOR_POWER);
                intakeServo.setPower(-SERVO_POWER);
                break;
            case STOPPED:
                intakeMotor.setPower(0);
                // If indexing is true, run servo even if motor is stopped
                intakeServo.setPower(indexing ? SERVO_POWER : 0);
                break;
        }

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

    public void intake() {currentState = State.INTAKING;}

    public void spit() {currentState = State.SPITTING;}

    public void stop() {currentState = State.STOPPED;}

    public void setSlow(boolean slow) {isSlowed = slow;}

    // --- RoadRunner Actions ---

    public Action actionIntake()
    {
        return packet -> {
            intake();
            return false; // Run once
        };
    }

    public Action actionStop()
    {
        return packet -> {
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
                if (startTime < 0) {startTime = System.currentTimeMillis() / 1000.0;}

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

    // Add these to the refactored Intake.java so your old code still compiles
    public void slow() {setSlow(true);}

    public void unslow() {setSlow(false);}

    public void unSpit() {intake();} // Or stop(), depending on what you want

    public void motorIntake() {intake();}

    public void motorSpit() {spit();}

    public void motorStop() {stop();}
}