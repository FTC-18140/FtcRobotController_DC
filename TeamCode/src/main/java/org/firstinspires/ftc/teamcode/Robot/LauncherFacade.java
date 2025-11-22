package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class LauncherFacade {
    private static final double JOYSTICK_SENSITIVITY = 1;
    // 1. Composition: Subsystems are now all internal
    private Turret turret;
    private Flywheel flywheel;
    private Limelight limelight; // Limelight is now part of the facade
    private Telemetry telemetry;

    // 2. Internal State Management
    private Pose2d robotPose; // Internal copy of the robot's pose

    // Target and alliance properties
    private Vector2d targetPos;
    private final Vector2d targetPosBlue = new Vector2d(60, 60);
    private final Vector2d targetPosRed = new Vector2d(60, -60); // Adjusted for typical field symmetry
    private ThunderBot2025.Alliance_Color allianceColor = ThunderBot2025.Alliance_Color.BLUE;

    // Constructor initializes all subsystems
    public void init(HardwareMap hwMap, Telemetry telem) {
        this.telemetry = telem;
        this.turret = new Turret();
        turret.init(hwMap, telem);
        this.flywheel = new Flywheel();
        flywheel.init(hwMap, telem);
        this.limelight = new Limelight(); // Initialize Limelight here
        limelight.init(hwMap, telem);

    }

    /**
     * Call this once per loop. It updates the robot's pose and all subsystems.
     * @param currentPose The most recent pose estimate from the drive train.
     */
    public void update(Pose2d currentPose) {
        this.robotPose = currentPose; // Update internal pose
        double distanceToGoal = getGoalDistance();

        turret.update();
        flywheel.update(distanceToGoal);
        limelight.update();
    }

    /**
     * Aims automatically using the best available sensor data.
     */
    public void aim() {
        // The standard aim is now just an augmented aim with zero augmentation.
        augmentedAim(0.0);
    }

    /**
     * Aims automatically but includes a manual joystick offset for driver correction.
     * @param joystickAugmentation The value from the joystick, which will be scaled and added.
     */
    public void augmentedAim(double joystickAugmentation) {
        // 1. Get the base auto-aim angle.
        double baseAutoAimAngle = getAutoAimAngle();

        // 2. Calculate the manual offset from the joystick value.
        double manualOffset = joystickAugmentation * JOYSTICK_SENSITIVITY;

        // 3. Calculate the final target angle.
        double finalTargetAngle = baseAutoAimAngle + manualOffset;

        // 4. Command the turret to seek to the final angle.
        turret.seekToAngle(finalTargetAngle);
    }

    // This method is now private, as it's an internal helper.
    private double getAutoAimAngle() {
        double difference = 0;
        if (limelight.hasTarget()) {
            telemetry.addData("Aiming Mode", "LIMELIGHT");
            double limelightXDegrees = limelight.getX();
            difference = limelightXDegrees * Turret.TURN_SPEED * Turret.TURRET_DEGREES_PER_SERVO_COMMAND;
        } else if (robotPose != null) {
            telemetry.addData("Aiming Mode", "ODOMETRY (Fallback)");
            Vector2d targetDirection = targetPos.minus(robotPose.position);
            double robotRelativeAngle = robotPose.heading.toDouble() - turret.getCurrentPosition() * (Math.PI / 2);
            difference = -targetDirection.angleCast().toDouble() - robotRelativeAngle;
        }
        else {
            telemetry.addData("Aiming Mode", "NO TARGET");
        }
        return turret.getCurrentPosition() + difference;
    }

    /** Prepares the flywheel for a shot based on the robot's current pose. */
    public void prepShot() {
        double distanceInches = getGoalDistance();
        double distanceMeters = distanceInches * 0.0254;

        double targetVelocity = flywheel.calculateBallVelocity(distanceMeters, 0.89, 60);
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

    /**
     * Returns a Road Runner Action that waits until the flywheel is at its target speed.
     * This action completes once the RPM is within the accepted tolerance.
     * @return An Action that can be used in a sequence.
     */
    public boolean isAtTargetRpm(){
        return flywheel.isAtTargetRpm();
    }
    public Action waitForChargeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // The action continues to run as long as the flywheel is NOT at its target RPM.
                // When flywheel.isAtTargetRpm() returns true, this expression becomes false,
                // and the action completes.
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

    /** Stops all launcher activity and puts subsystems into a safe state. */
    public void stop() {
        turret.holdPosition();
        flywheel.stop();
    }

    public void setAlliance(ThunderBot2025.Alliance_Color color) {
        this.allianceColor = color;
        this.targetPos = Objects.equals(this.allianceColor, "red") ? targetPosRed : targetPosBlue;
        limelight.setPipeline(Objects.equals(this.allianceColor, "red") ? 2 : 1);
    }

    // --- Private Helper Methods ---

    private void aimWithOdometry() {
        if (robotPose == null) return; // Safety check
        Vector2d targetDirection = targetPos.minus(robotPose.position);
        double robotRelativeAngle = robotPose.heading.toDouble() - turret.getCurrentPosition() * (Math.PI / 2);
        double angleDifference = targetDirection.angleCast().toDouble() - robotRelativeAngle;
        double newTurretTarget = turret.getCurrentPosition() - angleDifference;

        turret.seekToAngle(newTurretTarget);
    }

    private void aimWithLimelight() {
        double limelightXDegrees = limelight.getX(); // Get data directly from the internal object
        double difference = limelightXDegrees * Turret.TURN_SPEED * Turret.TURRET_DEGREES_PER_SERVO_COMMAND;
        double newTargetAngle = turret.getCurrentPosition() + difference;
        turret.seekToAngle(newTargetAngle);
    }

    private double getGoalDistance() {
        if (robotPose == null || targetPos == null) return 0;
        return targetPos.minus(robotPose.position).norm();
    }

    public boolean isAtTarget() {
        return turret.isAtTarget();
    }
}
