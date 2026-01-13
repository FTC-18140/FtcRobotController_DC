package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
@Config
public class Turret implements DataLoggable {

    // Define the states as an enum
    private enum State {
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
    //private DcMotor turretEnc;
    private PIDController turretAimPID;
    private Telemetry telemetry;

    // Tunable constants from your original file
    public static double P_TURRET = 0.005, I_TURRET = 0.02, D_TURRET = 0.0001, F_TURRET_MIN = 0.00, F_TURRET_MAX = 0.07;
    public static double MAX_TURRET_POS = 225;
    public static double MIN_TURRET_POS = -90;
    public static double TURRET_ANGLE_TOLERANCE = 2.5;

    public static double KV_ROT = 0.12; // Tunable: Gain for robot rotation
    public static double KV_TRANS = 0.15; // Tunable: Gain for translational apparent rotation
    public static boolean TELEM = true;

    public static double MAX_POWER = 0.8;
    public static double MIN_POWER = 0.045;

    public static double TURN_SPEED = 208.3; // From original lockOn
    public static double TURRET_DEGREES_PER_ENCODER_TICK = (double) 1 /8192 * 360 * 24.24/190.5;


    // State-specific variables
    private double targetAngle = 0;
    private double manualPower = 0;
    private double currentPosition = 0;
    //private double offsetAngle = 0;
    private double seekingPower = 0; // Member variable to be accessible for logging
    public static String STARTING_ANGLE = "TURRET_ENDING_ANGLE_AUTO";
    double startingAngle = (double) blackboard.getOrDefault(STARTING_ANGLE, (double) 0);
    public void init(HardwareMap hwMap, Telemetry telem) {


        currentPosition = startingAngle;
        this.telemetry = telem;
        turretAimPID = new PIDController(P_TURRET, I_TURRET, D_TURRET);
        try{
            turret = hwMap.dcMotor.get("turret");
            turret.setDirection(DcMotor.Direction.REVERSE);
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Motor\"turret\" not found", 0);
        }

    }

    public double getTargetPos(){
        return targetAngle;
    }

    // --- High-Level Commands to Change State ---

    /**
     *
     * @param angle in degrees
     */
    public void seekToAngle(double angle) {


        this.targetAngle = Range.clip(angle, MIN_TURRET_POS, MAX_TURRET_POS);
        this.currentState = State.SEEKING_ANGLE;
    }

    public void setManualPower(double power) {
        this.manualPower = power;
        this.currentState = State.MANUAL_CONTROL;
    }

//    public void setOffsetAngle(double angle) {
//        offsetAngle = angle;
//    }

    public void holdPosition() {
        this.targetAngle = this.currentPosition; // Hold where we are
        this.currentState = State.HOLDING;
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

        // 1. Static Feedforward (Wires/Friction)
        double ffStatic = Range.clip(Range.scale(currentPosition, -90, -30, F_TURRET_MAX, F_TURRET_MIN), F_TURRET_MIN, F_TURRET_MAX);

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
                if (currentState == State.SEEKING_ANGLE && isAtTarget()) {
                    this.currentState = State.HOLDING;
                }
                break;

            case MANUAL_CONTROL:
                setHardwarePower(manualPower);
                if (Math.abs(manualPower) < 0.05) holdPosition();
                break;
        }

        if ( TELEM ) {
            telemetry.addLine(" ------------- TURRET TELEM -------------");
            telemetry.addData("Turret Starting Angle: ", startingAngle);
            telemetry.addData("Turret Position", "%.2f", currentPosition);
            telemetry.addData("Turret Target", "%.2f", targetAngle);
            // Add these lines to see the "Blend" of control:
            telemetry.addData("PID Power", "%.3f", seekingPower);
            telemetry.addData("FF Total", "%.3f", (ffStatic * Math.signum(seekingPower)) + ffRobotRot + ffTrans);
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

        if (rSquared < 1e-6) return 0.0;

        // We need the world-frame velocity.
        // RoadRunner 1.0 PoseVelocity2d usually contains robot-relative v.
        // Convert to world-frame v for this calculation:
        double vxWorld = robotVel.linearVel.x * Math.cos(robotPose.heading.toDouble()) - robotVel.linearVel.y * Math.sin(robotPose.heading.toDouble());
        double vyWorld = robotVel.linearVel.x * Math.sin(robotPose.heading.toDouble()) + robotVel.linearVel.y * Math.cos(robotPose.heading.toDouble());

        // Cross product: (dx * vy - dy * vx) / r^2
        double translationalRate = (dx * vyWorld - dy * vxWorld) / rSquared;

        return translationalRate * KV_TRANS;
    }

    private void  setHardwarePower(double power) {
        if (power < 0 && currentPosition + power*45 <= MIN_TURRET_POS) {
            telemetry.addData("Turret position power override value: ", currentPosition + power*45);
            telemetry.addData("Turret Power sent to hardware: ", 0);
            turret.setPower(0);
        } else if (power > 0 && currentPosition + power*45 >= MAX_TURRET_POS) {
            telemetry.addData("Turret position power override value: ", currentPosition + power*45);
            telemetry.addData("Turret Power sent to hardware: ", 0);
            turret.setPower(0);
        } else {
            power = Range.clip(power, -MAX_POWER, MAX_POWER);
            if(power < 0){
                power = Range.scale(power, -MAX_POWER, 0, -MAX_POWER, -MIN_POWER);
            }else{
                power = Range.scale(power, 0, MAX_POWER, MIN_POWER, MAX_POWER);
            }
            telemetry.addData("Turret Power sent to hardware", power);
            turret.setPower(power);
        }
    }

    private void updateCurrentPosition() {
        this.currentPosition = turret.getCurrentPosition() * TURRET_DEGREES_PER_ENCODER_TICK + startingAngle;

    }
    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(this.currentPosition - targetAngle) < TURRET_ANGLE_TOLERANCE;
    }

    @Override
    public void logData(DataLogger logger) {
        logger.addField(this.targetAngle);
        logger.addField(this.currentPosition);
        logger.addField(P_TURRET);
        logger.addField(I_TURRET);
        logger.addField(D_TURRET);
        logger.addField(this.seekingPower);
    }
}
