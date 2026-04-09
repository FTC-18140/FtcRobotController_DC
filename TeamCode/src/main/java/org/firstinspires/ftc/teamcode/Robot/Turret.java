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
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config
public class Turret implements DataLoggable {

    // Define the states as an enum
    private enum State {
        STOP,
        HOLDING,
        SEEKING_ANGLE,
        MANUAL_CONTROL
    }

    public State getCurrentState() {
        return currentState;
    }

    private State currentState = State.HOLDING; // Initial state

    // Hardware and Utilities
    private DcMotor turret;
    private DcMotor turretEnc;
    private TouchSensor turretSwitch;
    private boolean isHomed;
    //private DcMotor turretEnc;
    private PIDController turretAimPID;
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P_TURRET = 0.024, I_TURRET = 0.022, D_TURRET = 0.001, F_TURRET_MIN = 0.0, F_TURRET_MAX = 0.018;
    public static double MAX_TURRET_POS = 270.0;
    public static double MIN_TURRET_POS = -90.0;
    public static double TURRET_ANGLE_TOLERANCE = 2.5;

    public static double KV_ROT = 0.17; // Tunable: Gain for robot rotation
    public static double KV_TRANS = 0.17; // Tunable: Gain for translational apparent rotation
    public static boolean TELEM = false;

    public static double MAX_POWER = 0.55;
    public static double MIN_POWER_POSITIVE = 0.015;
    public static double MIN_POWER_NEGATIVE = -0.015;

    public static double TURN_SPEED = 208.3; // From original lockOn
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 / 8192.0 * 360.0 * 24.24 / 190.5;


    // State-specific variables
    private double targetAngle = (double) 0;
    private double manualPower = (double) 0;
    public double manualAngle = (double) 0;
    private double currentPosition = (double) 0;
    private double currentDraw = 0.0;
    private double offsetAngle = (double) 0;
    private double seekingPower = (double) 0; // Member variable to be accessible for logging
    private double lastSeekingPower = (double) 0;
    public static String STARTING_ANGLE = "TURRET_ENDING_ANGLE_AUTO";
    double startingAngle;

    public void init(HardwareMap hwMap, Telemetry telem) {

        // touch sensor: Control hub Digital port 4
        startingAngle = (double) OpMode.blackboard.getOrDefault(STARTING_ANGLE, (double) 0);

        telemetry = telem;
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);
        try {
            turret = hwMap.dcMotor.get("turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            turretEnc = hwMap.dcMotor.get("leftFront");
            turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Motor\"turret\" not found", 0);
        }
        try {
            turretSwitch = hwMap.touchSensor.get("turretSwitch");
        } catch (Exception e) {
            telemetry.addData("Touch Sensor\"turretSwitch\" not found", 0);
        }


    }

    public void setStartAngle(double angle) {
        offsetAngle = -angle;
    }

    public double getTargetPos() {
        return targetAngle;
    }

    public void setOffset(double offset) {
        offsetAngle = offset;
    }

    // --- High-Level Commands to Change State ---

    /**
     * @param angle in degrees
     */
    void seekToAngle(double angle) {
        targetAngle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);
        currentState = State.SEEKING_ANGLE;
    }

    /**
     * Enforces mechanical rotation limits and manages the "Dead Zone" traversal.
     *
     * <p>This method ensures the turret stays within its physical bounds (-90° to 225°).
     * If a target is outside these bounds, it calculates if the target is reachable
     * by rotating "the long way" around the circle. If the target is in the unreachable
     * 135° gap, the turret clamps to the nearest hard stop.</p>
     *
     * @param angle The desired theoretical angle in degrees.
     * @return The physically possible angle in degrees, constrained to [-90, 225].
     */
    public double applyHardwareConstraints(double angle) {
        double finalAngle = angle % 360.0;

        // If target is below the right-side limit (-90)
        if (finalAngle < MIN_TURRET_POS) {
            // Check if rotating 360 degrees the other way puts us within the left limit (225)
            double altPath = finalAngle + 360.0;
            if (altPath <= MAX_TURRET_POS) {
                finalAngle = altPath;
            } else {
                finalAngle = MIN_TURRET_POS; // Goal is in the dead zone behind the robot
            }
        }
        // If target is above the left-side limit (225)
        else if (finalAngle > MAX_TURRET_POS) {
            // Check if rotating 360 degrees the other way puts us within the right limit (-90)
            double altPath = finalAngle - 360.0;
            if (altPath >= MIN_TURRET_POS && altPath <= MAX_TURRET_POS) {
                finalAngle = altPath;
            } else {
                finalAngle = MAX_TURRET_POS; // Goal is in the dead zone behind the robot
            }
        }
        return finalAngle;
    }

    public void setManualPower(double power) {
        manualPower = power;
        currentState = State.MANUAL_CONTROL;
    }

    public void holdPosition() {
        targetAngle = currentPosition; // Hold where we are
        currentState = State.STOP;
    }

    public boolean isHomed() {
        return isHomed;
    }

    /**
     * Main control loop for the turret. Calculates PID and multiple feedforward terms
     * to maintain a lock on the field-centric goal while the robot is in motion.
     *
     * <p>The control law combines:</p>
     * <ul>
     *     <li><b>PID:</b> Corrects positional error between current encoder position and target.</li>
     *     <li><b>Static FF:</b> Counteracts friction and wire harness pull based on position.</li>
     *     <li><b>Rotational FF (KV_ROT):</b> Cancels out the robot's own angular velocity.</li>
     *     <li><b>Translational FF (KV_TRANS):</b> Compensates for the apparent rotation of the
     *     target caused by the robot driving/strafing past it.</li>
     * </ul>
     *
     * @param robotPose Current field-centric pose from odometry.
     * @param robotVel  Current velocity from odometry (assumed robot-relative linear).
     * @param targetPos Field-centric coordinates of the high goal.
     */
    public void update(Pose2d robotPose,
                       PoseVelocity2d robotVel,
                       Vector2d targetPos) {
        updateCurrentPosition();
        turretAimPID.setPID(P_TURRET, I_TURRET, D_TURRET);

        isHomed = false;

        currentDraw = getTotalCurrentDraw();
        if (Flywheel.GOBILDA_MOTOR_STALL_CURRENT <= currentDraw) {
            telemetry.addData("TURRET STALLED", 0);
        }

        // 1. Static Feedforward (Wires/Friction)
        double ffStatic = Range.clip(Range.scale(currentPosition, -90.0, -15.0, F_TURRET_MAX, F_TURRET_MIN), F_TURRET_MIN, F_TURRET_MAX);

        // 2. Robot Rotation Feedforward
        double ffRobotRot = robotVel.angVel * KV_ROT;

        // 3. Robot Translation Feedforward
        double ffTrans = calculateTranslationalFF(robotPose, robotVel, targetPos);

        seekingPower = turretAimPID.calculate(currentPosition, targetAngle);

        // Combine all terms
        double totalPower = seekingPower + (ffStatic * Math.signum(seekingPower)) + ffRobotRot + ffTrans;

        switch (currentState) {
            case HOLDING:
            case SEEKING_ANGLE:
                setHardwarePower(totalPower);
                if (State.SEEKING_ANGLE == currentState && isAtTarget()) {
                    currentState = State.HOLDING;
                }
                break;

            case MANUAL_CONTROL:
                setHardwarePower(manualPower);
                if (0.01 > Math.abs(manualPower)) {
                    currentState = State.HOLDING;
                    setHardwarePower((double) 0);
                }
                ;
                break;
            case STOP:
                setHardwarePower((double) 0);
                break;
        }

        if (TELEM) {
            telemetry.addLine(" ------------- TURRET TELEM -------------");
            telemetry.addData("Turret Starting Angle: ", startingAngle);
            telemetry.addData("Turret Offset Angle: ", offsetAngle);
            telemetry.addData("Turret Position", "%.2f", currentPosition);
            telemetry.addData("Turret Target", "%.2f", targetAngle);
            // Add these lines to see the "Blend" of control:
            telemetry.addData("PID Power", "%.3f", seekingPower);
            telemetry.addData("FF Total", "%.3f", (ffStatic * Math.signum(seekingPower)) + ffRobotRot + ffTrans);
            telemetry.addData("Total current draw", "%.3f", getTotalCurrentDraw());
            telemetry.addData("Turret State", currentState);
        }

    }

    /**
     * Calculates the apparent rotational rate (rad/s) of a fixed field target relative
     * to the robot due to translational movement.
     *
     * <p>As the robot drives past a point, that point appears to "orbit" the robot.
     * This method uses the cross product of the relative position vector and the
     * velocity vector, divided by distance squared, to determine the angular velocity
     * required to maintain a lock without relying on PID error.</p>
     *
     * @param robotPose Current field-centric pose (used for world-frame conversion).
     * @param robotVel  Current robot-frame velocity.
     * @param targetPos The field-centric location of the goal.
     * @return The angular rate in rad/s, scaled by {@code KV_TRANS}.
     */
    private double calculateTranslationalFF(Pose2d robotPose,
                                            PoseVelocity2d robotVel,
                                            Vector2d targetPos) {

        double dx = targetPos.x - robotPose.position.x;
        double dy = targetPos.y - robotPose.position.y;
        double rSquared = dx * dx + dy * dy;

        if (1e-6 > rSquared) return 0.0;

        // We need the world-frame velocity.
        // RoadRunner 1.0 PoseVelocity2d usually contains robot-relative v.
        // Convert to world-frame v for this calculation:
        double vxWorld = robotVel.linearVel.x * Math.cos(robotPose.heading.toDouble()) - robotVel.linearVel.y * Math.sin(robotPose.heading.toDouble());
        double vyWorld = robotVel.linearVel.x * Math.sin(robotPose.heading.toDouble()) + robotVel.linearVel.y * Math.cos(robotPose.heading.toDouble());

        // Cross product: (dx * vy - dy * vx) / r^2
        double translationalRate = (dx * vyWorld - dy * vxWorld) / rSquared;

        return translationalRate * KV_TRANS;
    }

    private void setHardwarePower(double power) {

        double powerClipped = Range.clip(power, -MAX_POWER, MAX_POWER);
//            if (Math.signum(power) != Math.signum(lastSeekingPower)){
////                turretAimPID.reset();
//            }
        if ((double) 0 > powerClipped) {
            powerClipped = Range.scale(powerClipped, -MAX_POWER, (double) 0, -MAX_POWER, MIN_POWER_NEGATIVE);
        } else if ((double) 0 < powerClipped) {
            powerClipped = Range.scale(powerClipped, (double) 0, MAX_POWER, MIN_POWER_POSITIVE, MAX_POWER);
        }
//            telemetry.addData("Turret Power sent to hardware", power);
        turret.setPower(powerClipped);
        lastSeekingPower = powerClipped;
    }

    private void updateCurrentPosition() {
        currentPosition = (double) turretEnc.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK + startingAngle - offsetAngle;

    }

    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getCurrentPositionRaw() {
        return currentPosition + offsetAngle;
    }

    public double getTotalCurrentDraw() {
        DcMotorEx turretEx = (DcMotorEx) turret;
        return turretEx.getCurrent(CurrentUnit.AMPS);
    }


    public boolean isAtTarget() {
        return Math.abs(currentPosition - targetAngle) < TURRET_ANGLE_TOLERANCE;
    }

    @Override
    public void logData(DataLogger logger) {
        logger.addField(targetAngle);
        logger.addField(currentPosition);
        logger.addField(P_TURRET);
        logger.addField(I_TURRET);
        logger.addField(D_TURRET);
        logger.addField(seekingPower);
        logger.addField(currentDraw);
    }
}
