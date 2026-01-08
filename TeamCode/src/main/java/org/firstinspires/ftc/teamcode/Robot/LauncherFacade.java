package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
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
    // -------------------------------

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
    }

    public boolean isUsingLimelight() { return usingLimelight; }
    public double getLimelightX(){ return limelight.getX(); }

    /**
     * MAIN UPDATE LOOP
     * @param currentOdoPose The raw pose from RoadRunner drive.getPoseEstimate()
     */
    public void update(Pose2d currentOdoPose) {
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
//            double distToTag = Math.hypot(visionPose.position.x, visionPose.position.y);
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
        if (dt_h  > 180 ) {
            Pose2d fixedPose = new Pose2d(fusedPose.position.x, fusedPose.position.y, fusedPose.heading.toDouble() - 360);
            this.fusedPose = fixedPose;
        }
        else if (dt_h  < 180 ) {
            Pose2d fixedPose = new Pose2d(fusedPose.position.x, fusedPose.position.y, fusedPose.heading.toDouble() + 360);
            this.fusedPose = fixedPose;
        } else {
            this.fusedPose = currentOdoPose;
        }
        // ------------- End HOTFIX for AIMING

        // --- 5. RUN SUBSYSTEMS ---
        // Use fusedPose for distance calculation
        double distanceToGoal = getGoalDistance();

        turret.update();
        flywheel.update(distanceToGoal);

        telemetry.addData("Using Limelight: ", usingLimelight);
    }

    public void updateVision() { limelight.update(Math.toDegrees(fusedPose.heading.toDouble()) - getTurretAngle()); }
    public int getDetectedAprilTagId() { return limelight.id(); }
    public double getTurretAngle() { return turret.getCurrentPosition(); }
    public double getFlywheelRpm() { return flywheel.getCurrentRpm(); }
    public double getFlywheelTargetRpm() { return flywheel.getTargetRpm(); }

    public void aim() { augmentedAim(0.0); }

    public void augmentedAim(double joystickAugmentation) {
        // 1. Get the auto-aim angle based on FUSED POSE
        double baseAutoAimAngle = getAutoAimAngle();

        // 2. Calculate the manual offset
        double manualOffset = joystickAugmentation * JOYSTICK_SENSITIVITY;

        // 3. Final target
        double finalTargetAngle = baseAutoAimAngle + manualOffset;

        // 4. Send to Turret
        turret.seekToAngle(finalTargetAngle);

        // Telemetry for debugging
        telemetry.addData("Fused X", fusedPose.position.x);
        telemetry.addData("Fused Y", fusedPose.position.y);
        telemetry.addData("Aim Angle", finalTargetAngle);
    }

    /**
     * Calculates pure geometric angle from Fused Robot Pose to Goal
     */
    private double getAutoAimAngle() {
        double difference = 0;
        if (limelight.hasTarget()) {
            telemetry.addData("Aiming Mode", "LIMELIGHT");
            usingLimelight = true;
            double limelightXDegrees = limelight.getX();
            //difference = limelightXDegrees * Turret.TURN_SPEED * Turret.TURRET_DEGREES_PER_ENCODER_TICK;
            //difference = limelightXDegrees * Turret.TURRET_DEGREES_PER_ENCODER_TICK;
            difference = limelightXDegrees;

        } else if (fusedPose != null) {
            telemetry.addData("Aiming Mode", "ODOMETRY");
            usingLimelight = false;
            Vector2d targetDirection = targetPos.minus(fusedPose.position);
            //double robotRelativeAngle = -robotPose.heading.toDouble() + (turret.getCurrentPosition()) * (Math.PI/2);
            double robotRelativeAngle = Math.toDegrees(-fusedPose.heading.toDouble()) + turret.getCurrentPosition();
            difference = Math.toDegrees(-targetDirection.angleCast().toDouble()) - robotRelativeAngle;
        } else {
            telemetry.addData("Aiming Mode", "NO TARGET");
        }
        //return turret.getCurrentPosition() + difference / (Math.PI);
        return turret.getCurrentPosition() + difference;

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

    public void prepShot() {
        double distanceInches = getGoalDistance();
        double distanceMeters = distanceInches * 0.0254;
        double targetVelocity = flywheel.calculateBallVelocity(distanceMeters, 0.86, 48);
        double targetRpm = flywheel.calculateWheelRPM(targetVelocity);
        flywheel.setTargetRpm(targetRpm);
    }

    public Action prepShotAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                prepShot();
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
        limelight.setPipeline(Objects.equals(this.allianceColor, ThunderBot2025.Alliance_Color.RED) ? 2 : 1);
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
