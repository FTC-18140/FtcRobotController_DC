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

import java.util.Objects;

@Config
public class LauncherFacade implements DataLoggable {
    private static final double JOYSTICK_SENSITIVITY = 45.0;
    public static final double INCH_TO_METER = 0.0254;

    // Subsystems
    Turret turret = null;
    public Flywheel flywheel = null;
    private Limelight limelight = null;
    Telemetry telemetry = null;

    private boolean usingLimelight = false;

    public enum AimingMode {MAIN, ODOMETRY, LIMELIGHT, MANUAL, DIRECTIONAL}

    private AimingMode aimingMode = AimingMode.MAIN;

    // --- SENSOR FUSION VARIABLES ---
//    private KalmanPoseEstimator poseEstimator;
    private Pose2d fusedPose = new Pose2d(0, (double) 0, (double) 0); // This is the "Truth" we aim with
    private Pose2d lastOdoPose = null; // Used to calculate delta
    public static final double TURRET_OFFSET_X = 3.22805;
    public static final double TURRET_OFFSET_Y = -2.62074;
    public static final double LIMELIGHT_FORWARD_POSITION = 6.175;
    public Vector2d trueTargetVector = fusedPose.position;
    public static double trust = 0.0;

    private double smoothedTurretAngle = (double) 0;
    private boolean firstAimRun = true;
    public static double LPF_BETA = 1.0; // Higher value = more responsive

    // Target and alliance properties
    private Vector2d targetPos = null;
    private final Vector2d targetPosBlue = new Vector2d(69.0, 68.0);
    private final Vector2d targetPosRed = new Vector2d(70.0, -68.0);
    private ThunderBot2025.Alliance_Color allianceColor = ThunderBot2025.Alliance_Color.BLUE;

    public void init(HardwareMap hwMap, Telemetry telem, Pose2d startPose) {
        telemetry = telem;
        turret = new Turret();
        turret.init(hwMap, telem);
        flywheel = new Flywheel();
        flywheel.init(hwMap, telem);
        limelight = new Limelight();
        limelight.init(hwMap, telem);

        // Initialize Kalman Filter at (0,0,0) or load from file/auto transition
//        poseEstimator = new KalmanPoseEstimator(startPose);
        fusedPose = startPose;
        lastOdoPose = startPose;
    }

    public void setTurretStart(double angle) {
        turret.setStartAngle(angle);
    }

    public boolean isUsingLimelight() {
        return usingLimelight;
    }

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
        if (null == lastOdoPose) {
            lastOdoPose = currentOdoPose;
//            poseEstimator = new KalmanPoseEstimator(currentOdoPose);
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
//        poseEstimator.predict(globalDelta);
        lastOdoPose = currentOdoPose;

        // --- 3. MEASURE: Check Vision ---
//        telemetry.addData("Megatag2 Angle",Math.toDegrees(currentOdoPose.heading.toDouble()) - getTurretAngle());
        limelight.update(Math.toDegrees(currentOdoPose.heading.toDouble()) - getTurretAngle(), getTurretOffsetPosInRobotSpace());
        Vector2d visionPose = limelight.getMegaTagPose();
//        telemetry.addData("MT2 calculated Pose", visionPose);

        if (null != visionPose) {


            double distToTag = limelight.getDistance();
            if ((double) 0 > distToTag) {
                // Negative distToTag from the limelight method means it did not
                // see a valid AprilTag to use for the distance calculation.
                // Fallback to using the visionPose to calculate the distance.
                if (null != targetPos) {
                    distToTag = targetPos.minus(visionPose).norm();
                } else {
                    // Emergency fallback if we don't know alliance color yet
                    distToTag = 96.0; // Assume far away -> High uncertainty
                }
            }

            // Tuning: If > 48 inches away, start trusting vision significantly less
            // because depth accuracy drops off.
            double trustFactor = 1.0 + Math.pow(distToTag / 48.0, 2.0);

            //poseEstimator.update(visionPose, trustFactor);
//            usingLimelight = true;
        } else {
//            usingLimelight = false;
        }

        // --- 4. UPDATE FUSED POSE ---
        //this.fusedPose = poseEstimator.getFusedPose();

        // ------------- HOTFIX for AIMING
        fusedPose = currentOdoPose;
        // ------------- End HOTFIX for AIMING

        // --- 5. RUN SUBSYSTEMS ---
        // Use fusedPose for distance calculation
        getAutoAimAngle();
        double distanceToGoal = getGoalDistance();

        turret.update(fusedPose, currentOdoVelocity, targetPos);
        flywheel.update();

        telemetry.addData("Using Limelight: ", Boolean.valueOf(usingLimelight));
    }

    public void setAimingMode(AimingMode mode) {
        aimingMode = mode;
    }

    public AimingMode getAimingMode() {
        return aimingMode;
    }

    public void updateVision() {
        limelight.update(Math.toDegrees(fusedPose.heading.toDouble()) - getTurretAngle(), getTurretOffsetPosInRobotSpace());
    }

    public int getDetectedAprilTagId() {
        return limelight.id();
    }

    public double getTurretAngle() {
        return turret.getCurrentPosition();
    }

    public double getTurretAngleRaw() {
        return turret.getCurrentPositionRaw();
    }

    public double getFlywheelRpm() {
        return flywheel.getCurrentRpm();
    }

    public double getFlywheelTargetRpm() {
        return flywheel.getTargetRpm();
    }

    public double getFlywheelLowerBoundRpm() {
        return flywheel.getRpmLowerBound();
    }

    public double getFlywheelUpperBoundRpm() {
        return flywheel.getRpmUpperBound();
    }

    public void aim() {
        augmentedAim(0.0);
    }

    public boolean setTurretOffset() {
        // Calculate the vector (x, y) pointing from the robot to the goal
        boolean returnValue = false;
        aimingMode = AimingMode.DIRECTIONAL;
        if (turret.isHomed()) {
            holdTurretPosition();
            turret.setOffset(getTurretAngleRaw());
            returnValue = true;
        }
        return returnValue;
    }

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
            while (180.0 < delta) delta -= 360.0;
            while (-180.0 >= delta) delta += 360.0;

            // Apply the Low-Pass Filter (Complementary Filter)
            // smoothed = (OldValue) + (ShortestDelta * Beta)
            smoothedTurretAngle += (delta * LPF_BETA);
        }

        // --- HARDWARE CONSTRAINTS ---
        // Apply mechanical limits (-90 to 225) to the smoothed target.
        // This ensures the turret never tries to rotate through the "Dead Zone."
        double baseAngle = turret.applyHardwareConstraints(smoothedTurretAngle);

        // --- FINAL COMMAND ---
        // Combine the automated smoothed target with the manual joystick offset.
        double finalTargetAngle = baseAngle + (joystickAugmentation * JOYSTICK_SENSITIVITY);

        // Command the turret subsystem to the calculated angle.
        turret.seekToAngle(finalTargetAngle);

        // Diagnostic Telemetry
        double currentPosition = turret.getCurrentPosition();
        telemetry.addData("Turret Current", currentPosition);
        telemetry.addData("Turret Target", finalTargetAngle);
    }

    public void augmentedAimLimelight(double joystickAugmentation) {
        // 1. Get the raw "Instant" target from the best available sensor source.
        // This value is field-relative but normalized to be near the turret's current position.
        double instantTarget = getLimelightAimAngle();

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
            while (180.0 < delta) delta -= 360.0;
            while (-180.0 >= delta) delta += 360.0;

            // Apply the Low-Pass Filter (Complementary Filter)
            // smoothed = (OldValue) + (ShortestDelta * Beta)
            smoothedTurretAngle += (delta * LPF_BETA);
        }

        // --- HARDWARE CONSTRAINTS ---
        // Apply mechanical limits (-90 to 225) to the smoothed target.
        // This ensures the turret never tries to rotate through the "Dead Zone."
        double baseAngle = turret.applyHardwareConstraints(smoothedTurretAngle);

        // --- FINAL COMMAND ---
        // Combine the automated smoothed target with the manual joystick offset.
        double finalTargetAngle = baseAngle + (joystickAugmentation * JOYSTICK_SENSITIVITY);

        // Command the turret subsystem to the calculated angle.
        turret.seekToAngle(finalTargetAngle);

        // Diagnostic Telemetry
        double currentPosition = turret.getCurrentPosition();
        telemetry.addData("Turret Current", currentPosition);
        telemetry.addData("Turret Target", finalTargetAngle);
    }

    public void aimToAngleInFieldSpace(double angle) {
        double robotHeadingDouble = fusedPose.heading.toDouble();
        double robotHeadingDegrees = Math.toDegrees(robotHeadingDouble);
        double targetAngle = turret.applyHardwareConstraints(robotHeadingDegrees - angle);
        turret.seekToAngle(targetAngle);
    }

    public Vector2d getTurretOffsetPosInRobotSpace() {
        double robotHeading = fusedPose.heading.toDouble();
        return new Vector2d(
                TURRET_OFFSET_Y * Math.sin(-robotHeading) + (TURRET_OFFSET_X) * Math.cos(-robotHeading),
                TURRET_OFFSET_Y * Math.cos(-robotHeading) - (TURRET_OFFSET_X) * Math.sin(-robotHeading)
        );
    }


    public double getLimelightAimAngle() {
        double targetTurretAngle = getTurretAngleRaw();
        if (limelight.hasTarget()) {
            usingLimelight = true;
            double limelightAngle = getTurretAngleRaw() + limelight.getX();

            // Add the vision offset to the current physical encoder position.
            targetTurretAngle = limelightAngle;

            telemetry.addData("Aiming Mode LIMELIGHT -- target: ", "%.3f ", targetTurretAngle);
        } else {
            usingLimelight = false;
        }
        return targetTurretAngle;
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

        // --- 2. SENSOR PRIORITY: ODOMETRY ---
        // Fallback to Odometry if the Limelight is blocked or target is out of view.
        // We calculate the vector from our fused robot position to the field goal position.
        if (null != trueTargetVector && null != targetPos) {
            usingLimelight = false;


            // Vector from Turret offset pos to Goal
            trueTargetVector = targetPos.minus(fusedPose.position.plus(getTurretOffsetPosInRobotSpace()));

            // Calculate the absolute field-centric angle to the goal (Radians)
            double fieldAngleToGoal = Math.atan2(trueTargetVector.y, trueTargetVector.x);

            // HANDLE IMU WRAPPING:
            // We turn the raw angle into a Rotation2d and subtract our robot heading.
            // This yields the shortest relative distance from robot-front to goal,
            // automatically handling the jump across the +/- 180 degree line.
            double relativeAngleRad = Rotation2d.exp(fieldAngleToGoal).minus(fusedPose.heading);

            // Convert result to Degrees for the Turret Subsystem
            targetTurretAngle = -Math.toDegrees(relativeAngleRad);

            // --- NORMALIZATION LOGIC ---
            // Since the turret can go up to 225, a result of -170 (from the RR math)
            // is the same as 190. If the turret is currently at 180, we want to
            // go to 190, NOT -170.
            double currentTurret = turret.getCurrentPosition();


            telemetry.addData("Aiming Mode ODOMETRY -- target: ", " %.3f", targetTurretAngle);
            if (limelight.hasTarget()) {
                usingLimelight = true;
                double limeLightDistanceX = limelight.getDistance() * Math.sin(Math.toRadians(limelight.getX()));
                double limeLightDistanceY = -limelight.getDistance() * Math.cos(Math.toRadians(limelight.getX())) + LIMELIGHT_FORWARD_POSITION;

                double modifiedLimeLightX = Math.toDegrees(Math.atan2(limeLightDistanceX, ((double) 0 == limeLightDistanceY) ? 0.01 : limeLightDistanceY));

                double limelightAngle = turret.getCurrentPosition() - modifiedLimeLightX;

                // Add the vision offset to the current physical encoder position.
                targetTurretAngle = targetTurretAngle + trust * (limelightAngle - targetTurretAngle);

            }

            while (180.0 < targetTurretAngle - currentTurret) targetTurretAngle -= 360.0;
            while (-180.0 >= targetTurretAngle - currentTurret) targetTurretAngle += 360.0;
        }

        // --- 3. FALLBACK: IDLE ---
        // If no pose or target is available, hold current position to prevent erratic movement.
        else {
            telemetry.addData("Aiming Mode", "IDLE (No Target Found)");
            return turret.getCurrentPosition();
        }
        return targetTurretAngle;
    }


    private double getAutoAimAngleFUSION() {
        if (null == targetPos) return turret.getCurrentPosition();

        // Robot Heading (from fused pose)
        double robotHeading = fusedPose.heading.toDouble();

        //Offset Turret center of rotation
        Vector2d offsetPos = new Vector2d(
                TURRET_OFFSET_Y * Math.cos(-robotHeading) - (TURRET_OFFSET_X) * Math.sin(-robotHeading),
                TURRET_OFFSET_Y * Math.sin(-robotHeading) + (TURRET_OFFSET_X) * Math.cos(-robotHeading)
        );

        // Vector from Robot to Goal
        trueTargetVector = targetPos.minus(fusedPose.position.minus(offsetPos));

        // Absolute Field Angle to Goal (atan2 returns -PI to PI)
        double fieldAngleToGoal = Math.atan2(trueTargetVector.y, trueTargetVector.x);

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
        double distanceMeters = distanceInches * INCH_TO_METER;
        double targetVelocity = flywheel.calculateBallVelocity(distanceMeters, 0.6096, 48.0);
        double targetRpm = flywheel.calculateWheelRPM(targetVelocity);


        flywheel.setTargetRpm(targetRpm);
    }

    public void prepShotLow() {

        flywheel.setTargetRpm(Flywheel.MIN_SHOOTER_RPM);
    }

    public Action prepShotAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                prepShot();
                telemetry.addData("Flywheel Target RPM", flywheel.getTargetRpm());
                telemetry.addData("Flywheel RPM", flywheel.getCurrentRpm());

                return true;
            }
        };
    }

    public boolean isAtTargetRpm() {
        return flywheel.isAtTargetRpm();
    }

    public Action aimAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                aim();
                return !isAtTarget();
            }
        };
    }

    public Action pointToAction(double angle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                turret.seekToAngle(angle);
                return !isAtTarget();
            }
        };
    }

    public Action stopAction() {
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
        allianceColor = color;
        targetPos = Objects.equals(allianceColor, ThunderBot2025.Alliance_Color.RED) ? targetPosRed : targetPosBlue;
        limelight.setPipeline(Objects.equals(allianceColor, ThunderBot2025.Alliance_Color.RED) ? 2 : 1);
    }

    private double getGoalDistanceFUSION() {
        if (null == trueTargetVector || null == targetPos) return (double) 0;
        // Use FUSED pose for distance calculation
        telemetry.addData("distance: ", targetPos.minus(trueTargetVector).norm());
        return targetPos.minus(trueTargetVector).norm();
    }

    private double getGoalDistance() {
        double distance = trueTargetVector.norm();
        if (null == trueTargetVector || null == targetPos) distance = 0.0;
        // Use FUSED pose for distance calculation
        //        telemetry.addData("distance: ", distance);
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
        double headingDouble = fusedPose.heading.toDouble();
        logger.addField(headingDouble);
    }
}
