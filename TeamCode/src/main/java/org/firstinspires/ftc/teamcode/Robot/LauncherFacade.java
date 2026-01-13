package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.firstinspires.ftc.teamcode.Utilities.KalmanPoseEstimator;

import java.util.Objects;

@Config
public class LauncherFacade implements DataLoggable {
    private static final double JOYSTICK_SENSITIVITY = 45;

    // Subsystems
    private Turret turret;
    public Flywheel flywheel;
    private Limelight limelight;
    private Telemetry telemetry;

    private boolean usingLimelight = false;

    // --- SENSOR FUSION VARIABLES ---
    private KalmanPoseEstimator poseEstimator;
    private Pose2d fusedPose = new Pose2d(0, 0, 0); // This is the "Truth" we aim with
    private Pose2d lastOdoPose = null; // Used to calculate delta
    public static double TURRET_OFFSET_X = 3.5;
    public static double TURRET_OFFSET_Y = -4;
    public Vector2d turret_pos = fusedPose.position;

    private double smoothedTurretAngle = 0;
    private boolean firstAimRun = true;
    public static double LPF_BETA = 1.0; // Higher value = more responsive

    // Target and alliance properties
    private Vector2d targetPos;
    private final Vector2d targetPosBlue = new Vector2d(66, 66);
    private final Vector2d targetPosRed = new Vector2d(66, -66);
    private ThunderBot2025.Alliance_Color allianceColor = ThunderBot2025.Alliance_Color.BLUE;

    public void init(HardwareMap hwMap, Telemetry telem, Pose2d startPose) {
        this.telemetry = telem;
        this.turret = new Turret();
        turret.init(hwMap, telem);
        this.flywheel = new Flywheel();
        flywheel.init(hwMap, telem);
        this.limelight = new Limelight();
        limelight.init(hwMap, telem);

        // Initialize Kalman Filter at (0,0,0) or load from file/auto transition
        poseEstimator = new KalmanPoseEstimator(startPose);
        fusedPose = startPose;
        lastOdoPose = startPose;
    }

    public boolean isUsingLimelight() { return usingLimelight; }
    public double getLimelightX(){ return limelight.getX(); }
    public void setPipeline(int pipeline) {
        limelight.setPipeline(pipeline);
    }

    /**
     * MAIN UPDATE LOOP
     *
     * @param currentOdoPose     The raw pose from RoadRunner drive.getPoseEstimate()
     * @param currentOdoVelocity
     */
    public void update(Pose2d currentOdoPose, PoseVelocity2d currentOdoVelocity) {
        // --- 1. Calculate Odometry Delta ---
        if (lastOdoPose == null)  {
            lastOdoPose = currentOdoPose;
            poseEstimator = new KalmanPoseEstimator(currentOdoPose);
            return;
        }

        // --- 2. PREDICT: Calculate GLOBAL difference ---
        // We use global subtraction here because the Kalman Filter state vector
        // tracks global X/Y.
        double dt_x = currentOdoPose.position.x - lastOdoPose.position.x;
        double dt_y = currentOdoPose.position.y - lastOdoPose.position.y;

        // Handle heading wrap for the difference
        double dt_h = currentOdoPose.heading.minus(lastOdoPose.heading);
        Pose2d globalDelta = new Pose2d(dt_x, dt_y, dt_h);

        // Update Filter with the GLOBAL change
        poseEstimator.predict(globalDelta);
        lastOdoPose = currentOdoPose;

        // --- 3. MEASURE: Check Vision ---
        telemetry.addData("Megatag2 Angle",Math.toDegrees(currentOdoPose.heading.toDouble()) - getTurretAngle());
        limelight.update(Math.toDegrees(currentOdoPose.heading.toDouble()) - getTurretAngle());
        Pose2d visionPose = limelight.getMegaTagPose();

        if (visionPose != null) {
            // Determine trust based on distance (heuristic)
            double distToTag = limelight.getDistanceToTarget();
            if ( distToTag < 0)
            {
                // Negative distToTag from the limelight method means it did not
                // see a valid AprilTag to use for the distance calculation.
                // Fallback to using the visionPose to calculate the distance.
                if (targetPos != null) {
                    distToTag = targetPos.minus(visionPose.position).norm();
                } else {
                    // Emergency fallback if we don't know alliance color yet
                    distToTag = 96.0; // Assume far away -> High uncertainty
                }
            }

            // Tuning: If > 48 inches away, start trusting vision significantly less
            // because depth accuracy drops off.
            double trustFactor = 1.0 + Math.pow(distToTag / 48.0, 2);

            poseEstimator.update(visionPose, trustFactor);
            usingLimelight = true;
        } else {
            usingLimelight = false;
        }

        // --- 4. UPDATE FUSED POSE ---
        //this.fusedPose = poseEstimator.getFusedPose();

        // ------------- HOTFIX for AIMING
        fusedPose = currentOdoPose;
        // ------------- End HOTFIX for AIMING

        // --- 5. RUN SUBSYSTEMS ---
        // Use fusedPose for distance calculation
        double distanceToGoal = getGoalDistance();

        turret.update(fusedPose, currentOdoVelocity, targetPos);
        flywheel.update(distanceToGoal);

        telemetry.addData("Using Limelight: ", usingLimelight);
    }

    public void updateVision() { limelight.update(Math.toDegrees(fusedPose.heading.toDouble()) - getTurretAngle()); }
    public int getDetectedAprilTagId() { return limelight.id(); }
    public double getTurretAngle() { return turret.getCurrentPosition(); }
    public double getFlywheelRpm() { return flywheel.getCurrentRpm(); }
    public double getFlywheelTargetRpm() { return flywheel.getTargetRpm(); }

    public void aim() { augmentedAim(0.0); }

    /**
     * Updates the turret target using a blended approach of vision and odometry,
     * while allowing for real-time driver correction.
     *
     * <p>This method implements an Exponential Moving Average (EMA) filter to smooth
     * transitions when switching between sensor sources and handles the circular
     * shortest-path logic to ensure the turret reacts optimally to fast robot spins.</p>
     *
     * @param joystickAugmentation A normalized input (-1.0 to 1.0) from the driver
     *                             to manually offset the automated aim.
     */
    public void augmentedAim(double joystickAugmentation) {
        // 1. Get the raw "Instant" target from the best available sensor source.
        // This value is field-relative but normalized to be near the turret's current position.
        double instantTarget = getAutoAimAngle();

        if (firstAimRun) {
            // Initialize memory on the first loop to prevent the turret from
            // slowly "crawling" from 0 degrees at the start of the match.
            smoothedTurretAngle = instantTarget;
            firstAimRun = false;
        } else {
            // --- FILTER SHORT-PATH LOGIC ---
            // Calculate the delta between where we are and where we want to be.
            // We must normalize this delta to [-180, 180] so the filter always
            // moves the turret the shortest distance around the circle.
            double delta = instantTarget - smoothedTurretAngle;
            while (delta > 180) delta -= 360;
            while (delta <= -180) delta += 360;

            // Apply the Low-Pass Filter (Complementary Filter)
            // smoothed = (OldValue) + (ShortestDelta * Beta)
            smoothedTurretAngle += (delta * LPF_BETA);
        }

        // --- HARDWARE CONSTRAINTS ---
        // Apply mechanical limits (-90 to 225) to the smoothed target.
        // This ensures the turret never tries to rotate through the "Dead Zone."
        double baseAngle = applyHardwareConstraints(smoothedTurretAngle);

        // --- FINAL COMMAND ---
        // Combine the automated smoothed target with the manual joystick offset.
        double finalTargetAngle = baseAngle + (joystickAugmentation * JOYSTICK_SENSITIVITY);

        // Command the turret subsystem to the calculated angle.
        turret.seekToAngle(finalTargetAngle);

        // Diagnostic Telemetry
        telemetry.addData("Turret Current", turret.getCurrentPosition());
        telemetry.addData("Turret Target", finalTargetAngle);
    }

    /**
     * Calculates the theoretical target turret angle in degrees relative to the robot's front.
     * This method acts as the "Instantaneous Target" provider for the aiming system.
     *
     * <p>It provides a raw -180 to 180 degree target based on Limelight or Odometry,
     * and "unwraps" the result to be as close to the current turret position as possible
     * to support the turret's continuous linear encoder.</p>
     *
     * @return The raw target angle before smoothing or hardware clamping.
     */
    private double getAutoAimAngle() {
        double targetTurretAngle;

        // --- 1. SENSOR PRIORITY: LIMELIGHT ---
        // If the Limelight sees the target, we use the vision error (limelight.getX())
        // which represents the degrees the turret must turn from its CURRENT position
        // to center the goal in the camera frame.
        if (limelight.hasTarget()) {
            usingLimelight = true;

            // Add the vision offset to the current physical encoder position.
            targetTurretAngle = turret.getCurrentPosition() + limelight.getX();

            telemetry.addData("Aiming Mode LIMELIGHT -- target: ","%.3f ", targetTurretAngle);
        }

        // --- 2. SENSOR PRIORITY: ODOMETRY ---
        // Fallback to Odometry if the Limelight is blocked or target is out of view.
        // We calculate the vector from our fused robot position to the field goal position.
        else if (fusedPose != null && targetPos != null) {
            usingLimelight = false;

            // Calculate the vector (x, y) pointing from the robot to the goal
            Vector2d delta = targetPos.minus(fusedPose.position);

            // Calculate the absolute field-centric angle to the goal (Radians)
            double fieldAngleToGoal = Math.atan2(delta.y, delta.x);

            // HANDLE IMU WRAPPING:
            // We turn the raw angle into a Rotation2d and subtract our robot heading.
            // This yields the shortest relative distance from robot-front to goal,
            // automatically handling the jump across the +/- 180 degree line.
            double relativeAngleRad = Rotation2d.exp(fieldAngleToGoal).minus(fusedPose.heading);

            // Convert result to Degrees for the Turret Subsystem
            targetTurretAngle = Math.toDegrees(relativeAngleRad);

            // --- NORMALIZATION LOGIC ---
            // Since the turret can go up to 225, a result of -170 (from the RR math)
            // is the same as 190. If the turret is currently at 180, we want to
            // go to 190, NOT -170.
            double currentTurret = turret.getCurrentPosition();
            while (targetTurretAngle - currentTurret > 180)  targetTurretAngle -= 360;
            while (targetTurretAngle - currentTurret <= -180) targetTurretAngle += 360;

            telemetry.addData("Aiming Mode ODOMETRY -- target: "," %.3f", targetTurretAngle);
        }

        // --- 3. FALLBACK: IDLE ---
        // If no pose or target is available, hold current position to prevent erratic movement.
        else {
            telemetry.addData("Aiming Mode", "IDLE (No Target Found)");
            return turret.getCurrentPosition();
        }
        return targetTurretAngle;
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
    private double applyHardwareConstraints(double angle) {
        double finalAngle = angle;

        // If target is below the right-side limit (-90)
        if (finalAngle < -90) {
            // Check if rotating 360 degrees the other way puts us within the left limit (225)
            double altPath = finalAngle + 360;
            if (altPath <= 225) {
                finalAngle = altPath;
            } else {
                finalAngle = -90; // Goal is in the dead zone behind the robot
            }
        }
        // If target is above the left-side limit (225)
        else if (finalAngle > 225) {
            // Check if rotating 360 degrees the other way puts us within the right limit (-90)
            double altPath = finalAngle - 360;
            if (altPath >= -90) {
                finalAngle = altPath;
            } else {
                finalAngle = 225; // Goal is in the dead zone behind the robot
            }
        }
        return finalAngle;
    }

    private double getAutoAimAngleFUSION() {
        if (targetPos == null) return turret.getCurrentPosition();

        // Robot Heading (from fused pose)
        double robotHeading = this.fusedPose.heading.toDouble();

        //Offset Turret center of rotation
        Vector2d offsetPos = new Vector2d(
                TURRET_OFFSET_Y * Math.cos(-robotHeading) - (-TURRET_OFFSET_X) * Math.sin(-robotHeading),
                TURRET_OFFSET_Y * Math.sin(-robotHeading) + (-TURRET_OFFSET_X) * Math.cos(-robotHeading)
        );

        // Vector from Robot to Goal
        this.turret_pos = targetPos.minus(this.fusedPose.position.minus(offsetPos));

        // Absolute Field Angle to Goal (atan2 returns -PI to PI)
        double fieldAngleToGoal = Math.atan2(turret_pos.y, turret_pos.x);

        // Relative Angle = FieldAngle - RobotHeading
        double relativeAngleRad = robotHeading - fieldAngleToGoal;

        // Convert to degrees
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

//        // Normalize to Turret's range so the turret takes shortest path
//        while (relativeAngleDeg > Turret.MAX_TURRET_POS) relativeAngleDeg -= 360;
//        while (relativeAngleDeg < Turret.MIN_TURRET_POS) relativeAngleDeg += 360;

        // Note: You might need to add turret.getCurrentPosition() offset here depending
        // on if your turret acts in absolute mode or relative mode.
        // Based on previous code: "seekToAngle" seemed to take a relative target?
        // If seekToAngle expects -90 to 90 relative to ROBOT FRONT, return relativeAngleDeg.

        return relativeAngleDeg;
    }
    public void holdTurretPosition() {
        turret.holdPosition();
    }

    public void prepShot() {
        double distanceInches = getGoalDistance();
        double distanceMeters = distanceInches * 0.0254;
        double targetVelocity = flywheel.calculateBallVelocity(distanceMeters, 0.86, 48);
        double targetRpm = flywheel.calculateWheelRPM(targetVelocity);

        telemetry.addData("Distance Meters: ", distanceMeters);
        telemetry.addData("Target Velocity: ", targetVelocity);
        flywheel.setTargetRpm(targetRpm);
    }

    public Action prepShotAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                prepShot();
                telemetry.addData("Flywheel Target RPM", flywheel.getTargetRpm());
                telemetry.addData("Flywheel RPM", flywheel.getCurrentRpm());

                return false;
            }
        };
    }

    public boolean isAtTargetRpm(){ return flywheel.isAtTargetRpm(); }

    public Action waitForChargeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return !flywheel.isAtTargetRpm();
            }
        };
    }

    public Action aimAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                aim();
                return !isAtTarget();
            }
        };
    }

    public Action pointToAction( double angle ){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.seekToAngle(angle);
                return !isAtTarget();
            }
        };
    }

    public Action stopAction(  ){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flywheel.stop();
                return false;
            }
        };
    }

    public void setTurretManualPower(double power) {
        turret.setManualPower(power);
    }

    public void stop() {
        flywheel.stop();
    }

    public void setAlliance(ThunderBot2025.Alliance_Color color) {
        this.allianceColor = color;
        this.targetPos = Objects.equals(this.allianceColor, ThunderBot2025.Alliance_Color.RED) ? targetPosRed : targetPosBlue;
//        limelight.setPipeline(Objects.equals(this.allianceColor, ThunderBot2025.Alliance_Color.RED) ? 2 : 1);
    }

    private double getGoalDistanceFUSION() {
        if (turret_pos == null || targetPos == null) return 0;
        // Use FUSED pose for distance calculation
        telemetry.addData("distance: ", targetPos.minus(turret_pos).norm());
        return targetPos.minus(turret_pos).norm();
    }
    private double getGoalDistance() {
        if (fusedPose == null || targetPos == null) return 0;
        // Use FUSED pose for distance calculation
        double distance = targetPos.minus(fusedPose.position).norm();
        telemetry.addData("distance: ", distance);
        return distance;
    }

    public boolean isAtTarget() {
        return turret.isAtTarget();
    }

    @Override
    public void logData(DataLogger logger) {
        limelight.logData(logger);
        turret.logData(logger);
        // Log fusion debug info
        logger.addField(fusedPose.position.x);
        logger.addField(fusedPose.position.y);
        logger.addField(fusedPose.heading.toDouble());
    }
}
