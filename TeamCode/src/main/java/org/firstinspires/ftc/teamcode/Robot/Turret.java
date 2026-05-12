package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Auto.AutoRedDepot_12;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.ThresholdMotor;

@Config
public class Turret
{

    private enum State
    {STOP, HOLDING, SEEKING_ANGLE, MANUAL_CONTROL}

    // --- CORE HARDWARE & UTILITIES ---
    private DcMotorEx turret = null;
    private ThresholdMotor turretWriter = null;
    private DcMotorEx turretEnc = null;
    private TouchSensor turretSwitch = null;
    private PIDController turretAimPID = null;
    private Telemetry telemetry = null;

    // --- CORE CONSTANTS (Keepers) ---
    public static double P_TURRET = 0.0136, I_TURRET = 0.0, D_TURRET = 0.001645;
    public static double F_STATIC = 0.035;
    public static double KV_ROT = 0.27;
    public static double KV_TRANS = 0.12;
    public static double TURRET_POWER_THRESHOLD = 0.005;
    public static double MAX_POWER = 0.6;
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 / 8192.0 * 360.0 * 16 / 100;

    // --- LIMITS & TOLERANCES ---
    public static double MIN_TURRET_POS = -90;
    public static double MAX_TURRET_POS = 360 + MIN_TURRET_POS;
    public static double TURRET_ANGLE_TOLERANCE = 3.5;
    public static double TURRET_ANGLE_SOFT_TOLERANCE = 3.5;

    // --- LEGACY / EXPERIMENTAL CONSTANTS (Candidates for removal) ---
    public static double OFFSET_FOR_WEIRD_STUFF = 3.5; // <--- The "Weird Stuff" is here
    public static double F_ACCEL = 0.0011, F_ACCEL_MAX = 0.0093;
    public static double F_RES_POS_MIN = 0.01, F_RES_POS_MAX = 0.05;
    public static double F_RES_NEG_MIN = 0.05, F_RES_NEG_MAX = 0.095;
    public static double F_RANGE_MIN = 20, F_RANGE_MAX = 180;
    public static double F_LAUNCHING = 0.09;
    public static boolean TELEM = false;

    // --- STATE VARIABLES ---
    private State currentState = State.HOLDING;
    private double targetAngle = 0.0;
    private double lastTargetAngle = 0.0;
    private double manualPower = 0.0;
    private double currentPosition = 0.0;
    private double offsetAngle = 0.0;
    private double startingAngle = 0.0;
    private boolean isHomed = false;
    public static String STARTING_ANGLE_KEY = AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        telemetry = telem;
        startingAngle = (double) OpMode.blackboard.getOrDefault(STARTING_ANGLE_KEY, (double) 0);
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);

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

        try
        {
            turretSwitch = hwMap.touchSensor.get("turretSwitch");
        }
        catch (RuntimeException e)
        {
            telemetry.addData("turretSwitch not found", e.getMessage());
        }
    }

    /**
     * Main Control Loop
     */
    public void update(Pose2d robotPose, PoseVelocity2d robotVel, Vector2d targetPos, boolean launching)
    {
        updateCurrentPosition();
        isHomed = turretSwitch.isPressed();

        // 1. PID Calculation
        turretAimPID.setPID(P_TURRET, I_TURRET, D_TURRET);
        double pidPower = turretAimPID.calculate(currentPosition, targetAngle);

        // 2. Feedforward Calculation (Isolated for future simplification)
        double ffPower = calculateTotalFeedforward(robotPose, robotVel, targetPos, pidPower, launching);

        // 3. Combine and Apply via State Machine
        double totalPower = pidPower + ffPower;
        handleStateEngine(totalPower);

        if (TELEM)
        {
            doTelemetry(pidPower, ffPower);
        }

        lastTargetAngle = targetAngle;
    }

    private void updateCurrentPosition()
    {
        // Calculation isolated here. Weird stuff is subtracted at the end.
        double rawDegrees = -turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK;
        currentPosition = rawDegrees + startingAngle - offsetAngle - OFFSET_FOR_WEIRD_STUFF;
    }

    /**
     * Consolidates all 5+ Feedforward terms.
     * You can "unplug" terms here later without breaking the main loop.
     */
    private double calculateTotalFeedforward(Pose2d pose, PoseVelocity2d vel, Vector2d target, double pidPower, boolean launching)
    {
        double targetShift = targetAngle - lastTargetAngle;
        double errorAbs = Math.abs(targetAngle - currentPosition);
        double mediumErrorScalar = (errorAbs * errorAbs) / (TURRET_ANGLE_SOFT_TOLERANCE * TURRET_ANGLE_SOFT_TOLERANCE);

        // Term A: Static Friction
        double ffStatic = F_STATIC * Math.signum(pidPower);

        // Term B: Robot Motion Compensation (Rotation + Translation)
        double ffRobotRot = vel.angVel * KV_ROT;
        double ffTrans = calculateTranslationalFF(pose, vel, target);

        // Term C: Experimental Acceleration FF
        double ffAccel = (0.025 < targetShift ? Math.min(F_ACCEL / targetShift, F_ACCEL_MAX) : 0);

        // Term D: Experimental Resistance FF (Wires/Tension)
        double ffResistance = 0;
        if (targetAngle > F_RANGE_MAX)
        {
            ffResistance = Range.scale(Range.clip(targetAngle, F_RANGE_MAX, MAX_TURRET_POS), F_RANGE_MAX, MAX_TURRET_POS, F_RES_POS_MIN, F_RES_POS_MAX);
        }
        else if (targetAngle < F_RANGE_MIN)
        {
            ffResistance = Range.scale(Range.clip(targetAngle, F_RANGE_MIN, MIN_TURRET_POS), F_RANGE_MIN, MIN_TURRET_POS, F_RES_NEG_MIN, F_RES_NEG_MAX);
        }
        ffResistance *= Math.signum(pidPower);

        // Term E: Launching Compensation
        double ffLaunch = (launching && currentPosition < targetAngle ? F_LAUNCHING * Math.min(mediumErrorScalar, 1) : 0);

        return ffStatic + ffRobotRot + ffTrans + ffAccel + ffResistance + ffLaunch;
    }

    private void handleStateEngine(double totalPower)
    {
        double errorAbs = Math.abs(targetAngle - currentPosition);
        double lowerErrorScalar = (errorAbs * errorAbs) / (TURRET_ANGLE_TOLERANCE * TURRET_ANGLE_TOLERANCE);

        switch (currentState)
        {
            case HOLDING:
            case SEEKING_ANGLE:
                if (isAtTarget())
                {
                    setHardwarePower(totalPower * lowerErrorScalar);
                    currentState = State.HOLDING;
                }
                else
                {
                    setHardwarePower(totalPower);
                    currentState = State.SEEKING_ANGLE;
                }
                break;
            case MANUAL_CONTROL:
                setHardwarePower(manualPower);
                if (Math.abs(manualPower) < 0.01) {currentState = State.HOLDING;}
                break;
            case STOP:
                setHardwarePower(0);
                break;
        }
    }

    private double calculateTranslationalFF(Pose2d robotPose, PoseVelocity2d robotVel, Vector2d targetPos)
    {
        double dx = targetPos.x - robotPose.position.x;
        double dy = targetPos.y - robotPose.position.y;
        double rSquared = dx * dx + dy * dy;
        if (rSquared < 1e-6) {return 0.0;}

        double vxWorld = robotVel.linearVel.x * Math.cos(-robotPose.heading.toDouble()) - robotVel.linearVel.y * Math.sin(-robotPose.heading.toDouble());
        double vyWorld = robotVel.linearVel.x * Math.sin(-robotPose.heading.toDouble()) + robotVel.linearVel.y * Math.cos(-robotPose.heading.toDouble());

        return ((dx * vyWorld - dy * vxWorld) / rSquared) * KV_TRANS;
    }

    private void doTelemetry(double pidPower, double ffTotal)
    {
        telemetry.addLine(" ------------- TURRET TELEM -------------");
        telemetry.addData("Turret Position", "%.2f", currentPosition);
        telemetry.addData("Turret Target", "%.2f", targetAngle);
        telemetry.addData("PID Power", "%.3f", pidPower);
        telemetry.addData("FF Total", "%.3f", ffTotal);
        telemetry.addData("Turret State", currentState);
    }

    // --- PUBLIC API ---

    public void seekToAngle(double angle)
    {
        targetAngle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);
        currentState = State.SEEKING_ANGLE;
    }

    public void setManualPower(double power)
    {
        manualPower = power;
        currentState = State.MANUAL_CONTROL;
    }

    public void holdPosition()
    {
        targetAngle = currentPosition;
        currentState = State.SEEKING_ANGLE;
    }

    private void setHardwarePower(double power)
    {
        if (turretWriter != null) {turretWriter.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));}
    }

    public double getCurrentPosition() {return currentPosition;}

    public boolean isAtTarget() {return Math.abs(currentPosition - targetAngle) < TURRET_ANGLE_TOLERANCE;}

    public void setStartAngle(double angle) {offsetAngle = -angle;}

    public void zeroTurret() {turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
}
