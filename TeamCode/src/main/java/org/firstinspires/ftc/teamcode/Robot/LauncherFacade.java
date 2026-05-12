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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;

import java.util.Objects;

@Config
public class LauncherFacade
{

    // Subsystems
    private Turret turret = null;
    public FlywheelController flywheel = null;
    private Limelight limelight = null;
    private Telemetry telemetry = null;


    public enum AimingMode
    {MAIN, ODOMETRY, LIMELIGHT, MANUAL, DIRECTIONAL}

    private AimingMode aimingMode = AimingMode.MAIN;

    private Pose2d fusedPose = new Pose2d(0.0, 0.0, 0.0); // This is the "Truth" we aim with
    private Pose2d lastOdoPose = null; // Used to calculate delta
    public static double TURRET_OFFSET_X = -0.94488;
    public static double TURRET_OFFSET_Y = -3.04528;

    private Vector2d inertiaOffset = null;
    Vector2d offsetTarget = new Vector2d(0, 0);
    public static double INERTIA_FACTOR = 0.25;
    private double last_time_ms = 0;
    private Vector2d trueTargetVector = fusedPose.position;

    private double smoothedTurretAngle = 0.0;
    private boolean firstAimRun = true;
    public static boolean TELEM = false;
    public static double LPF_BETA = 1.0; // Higher value = more responsive

    // Target and alliance properties
    private Vector2d targetPos = null;
    private static final Vector2d targetPosBlue = new Vector2d(67.0, 67.0);
    private static final Vector2d targetPosRed = new Vector2d(67.0, -67.0);
    private ThunderBot2025.Alliance_Color allianceColor = ThunderBot2025.Alliance_Color.BLUE;
    public double fieldAngleToGoal = 0.0;

    public void init(HardwareMap hwMap, Telemetry telem, Pose2d startPose)
    {
        telemetry = telem;
        turret = new Turret();
        turret.init(hwMap, telem);

        flywheel = new FlywheelController();
        flywheel.init(hwMap, telem);

        limelight = new Limelight();
        limelight.init(hwMap, telem);

        fusedPose = startPose;
        lastOdoPose = startPose;
    }

    public void setTurretStart(double angle)
    {
        turret.setStartAngle(angle);
    }

    public void setPipeline(int pipeline)
    {
        limelight.setPipeline(pipeline);
    }

    /**
     * MAIN UPDATE LOOP
     * Orchestrates the calculation of target angles and the commanding of subsystems (turret, flywheel).
     * This method ensures the turret is always aiming based on the current mode.
     *
     * @param currentOdoPose     The raw pose from RoadRunner drive.getPoseEstimate()
     * @param currentOdoVelocity The raw velocity from RoadRunner drive.getPoseVelocity()
     * @param voltage            Current battery voltage for motor compensation
     * @param launching          True if the indexer is actively launching, affects turret feedforward
     */
    public void update(Pose2d currentOdoPose, PoseVelocity2d currentOdoVelocity, double voltage, boolean launching)
    {
        // Handle first loop where lastOdoPose is not yet initialized
        if (null == lastOdoPose)
        {
            lastOdoPose = currentOdoPose;
            // Early exit if this is the very first update and there's no prior pose to calculate delta
            return;
        }

        // Calculate update rate for telemetry/debug
        double update_rate_seconds = (System.currentTimeMillis() - last_time_ms) / 1000;
        last_time_ms = System.currentTimeMillis(); // Update last_time_ms at the beginning of the loop for accurate delta

        // Update fused pose and calculate inertia offset
        fusedPose = currentOdoPose;
        // The inertiaOffset and offsetTarget are used by getAutoAimAngle() and turret.update()
        inertiaOffset = currentOdoVelocity.linearVel.times(INERTIA_FACTOR);
        offsetTarget = targetPos.minus(inertiaOffset); // targetPos is set by setAlliance()

        // Calculate the instantaneous target angle for the turret
        // This method also updates internal state like 'fieldAngleToGoal' and 'trueTargetVector'
        double instantTargetAngle = getAutoAimAngle();
        double distanceToGoal = getGoalDistance();

        // --- Turret Control Logic based on AimingMode ---
        switch (aimingMode) {
            case MAIN:
            case ODOMETRY: // Assuming these are auto-aiming modes
            case LIMELIGHT:
                // Auto-aiming: Apply smoothing and command the turret
                processAutoAiming(instantTargetAngle);
                break;
            case MANUAL:
            case DIRECTIONAL:
                // In manual modes, the turret is expected to be controlled directly by
                // methods like `setTurretManualPower` or `aimToAngleInFieldSpace`
                // (which set the Turret's internal state).
                // If the driver is NOT actively providing input (e.g., joystick is centered),
                // we should command the turret to hold its last position to prevent drift.
                // The `turret.update()` method (called below) will then execute the hold.
                if (turret.getCurrentState() != Turret.State.MANUAL_CONTROL) {
                    turret.holdPosition();
                }
                break;
        }

        // --- Subsystem Updates (Always run regardless of aiming mode) ---
        // The Turret subsystem's update method processes sensor inputs,
        // PID calculations, and feedforward terms. Its actual motor output
        // is governed by the state (SEEKING_ANGLE, MANUAL_CONTROL, HOLDING, STOP)
        // that was set by the command in the switch statement above (e.g., seekToAngle, holdPosition, setManualPower).
        turret.update(fusedPose, currentOdoVelocity, offsetTarget, launching);

        // Flywheel subsystem updates are always active.
        flywheel.update(currentOdoVelocity, Math.toDegrees(fieldAngleToGoal), voltage, distanceToGoal);

        // --- Telemetry ---
        if (TELEM)
        {
            telemetry.addData("update rate (seconds): ", update_rate_seconds);
            telemetry.addData("Launcher Aiming Mode", aimingMode.name()); // Useful for debugging mode changes
        }
    }

    // --- Private helper method for auto-aiming logic ---
    /**
     * Processes the target angle, applies smoothing, and commands the turret subsystem.
     * This logic was previously inside the public `aim()` method.
     * @param instantTarget The raw, instantaneous target angle from `getAutoAimAngle()`.
     */
    private void processAutoAiming(double instantTarget) {
        if (firstAimRun)
        {
            // Initialize memory on the first loop to prevent the turret from
            // slowly "crawling" from 0 degrees at the start of the match.
            smoothedTurretAngle = instantTarget;
            firstAimRun = false;
        }
        else
        {
            // --- FILTER SHORT-PATH LOGIC ---
            // Calculate the delta between where we are and where we want to be.
            // We must normalize this delta to [-180, 180] so the filter always
            // moves the turret the shortest distance around the circle.
            double delta = instantTarget - smoothedTurretAngle;
            while (180.0 < delta)
            {
                delta -= 360.0;
            }
            while (-180.0 >= delta)
            {
                delta += 360.0;
            }

            // Apply the Low-Pass Filter (Complementary Filter)
            // smoothed = (OldValue) + (ShortestDelta * Beta)
//            smoothedTurretAngle += (delta * LPF_BETA);
            smoothedTurretAngle = instantTarget; // drop low pass filtering for now...
        }
        double baseAngle = turret.applyHardwareConstraints(smoothedTurretAngle);

        // --- FINAL COMMAND ---
        // Command the turret subsystem to the calculated angle.
        // This sets the Turret's target and internal state (e.g., SEEKING_ANGLE).
        turret.seekToAngle(baseAngle);

        // Telemetry specific to aiming
        if (TELEM)
        {
            // Note: turret.getCurrentPosition() is updated in Turret.update(), which is called later
            // in LauncherFacade.update(). So this might show slightly old data if telemetry
            // is printed before Turret.update() has run in the *current* cycle.
            // For general debugging, it's usually fine.
            telemetry.addData("Turret Current (smoothed)", smoothedTurretAngle); // More relevant to LPF output
            telemetry.addData("Turret Target (commanded)", baseAngle);
        }
    }


    public void setAimingMode(AimingMode mode)
    {
        // Reset the LPF when changing modes to prevent "jumps" if auto-aim starts again
        // Only reset if switching TO an auto-aim mode from a different mode.
        if (aimingMode != mode && (mode == AimingMode.MAIN || mode == AimingMode.ODOMETRY || mode == AimingMode.LIMELIGHT)) {
            firstAimRun = true;
        }
        aimingMode = mode;
    }

    public AimingMode getAimingMode()
    {
        return aimingMode;
    }

    int getDetectedAprilTagId()
    {
        return limelight.id();
    }

    public double getTurretAngle()
    {
        return turret.getCurrentPosition();
    }

    private double getTurretAngleRaw()
    {
        return turret.getCurrentPositionRaw();
    }

    public double getLowerFlywheelRpm()
    {
        return flywheel.getLowerFlywheelCurrentRPM();
    }

    public double getUpperFlywheelRpm()
    {
        return flywheel.getUpperFlywheelCurrentRPM();
    }

    public double getFlywheelTargetRpm()
    {
        return flywheel.getLowerFlywheelTargetRPM();
    }

    public double getUpperFlywheelTargetRpm()
    {
        return flywheel.getUpperFlywheelTargetRPM();
    }

    double getFlywheelLowerBoundRpm()
    {
        return Flywheel.RPM_LOWER_BOUND;
    }

    double getFlywheelUpperBoundRpm()
    {
        return Flywheel.RPM_UPPER_BOUND;
    }

    public double getTotalCurrentDraw()
    {
        return turret.getTotalCurrentDraw() + flywheel.getTotalCurrentDraw();
    }

    /**
     * This method is now primarily for compatibility with RoadRunner Actions or explicit one-shot commands.
     * For continuous aiming, the `update()` method handles it based on `aimingMode`.
     */
    public void aim()
    {
        // Ensure auto-aim mode is active if aim() is called directly.
        if (aimingMode != AimingMode.MAIN) {
            setAimingMode(AimingMode.MAIN);
        }
        processAutoAiming(getAutoAimAngle());
    }

    boolean setTurretOffset()
    {
        boolean returnValue = false;
        // If this is a calibration requiring manual movement, ensure the mode allows it.
        // It's crucial this doesn't conflict with ongoing auto-aim.
        // If setTurretOffset implies a temporary manual override for calibration,
        // you might want to explicitly set a temporary manual mode here or handle it externally.
        // For now, removing the mode change from inside this method to prevent it from
        // overriding the driver's choice of aimingMode unintentionally.
        if (turret.isHomed())
        {
            double offset = getTurretAngleRaw();
            turret.setOffsetAngle(offset);
            returnValue = true;
        }
        return returnValue;
    }

    /**
     * Commands the turret to a specific angle in field space. This implies manual control.
     * @param angle The desired field-centric angle in degrees.
     */
    public void aimToAngleInFieldSpace(double angle)
    {
        // When this method is called, it signifies a direct manual command, so switch to MANUAL mode.
        if (aimingMode != AimingMode.MANUAL) {
            setAimingMode(AimingMode.MANUAL);
        }
        double robotHeadingDouble = fusedPose.heading.toDouble();
        double robotHeadingDegrees = Math.toDegrees(robotHeadingDouble);
        // The turret `applyHardwareConstraints` must be applied to the target *relative to the robot*.
        double targetAngleRobotRelative = turret.applyHardwareConstraints(robotHeadingDegrees - angle);
        turret.seekToAngle(targetAngleRobotRelative);
    }

    private Vector2d getTurretOffsetPosInRobotSpace()
    {
        double robotHeading = fusedPose.heading.toDouble();
        return new Vector2d(
                TURRET_OFFSET_Y * Math.sin(-robotHeading) + (TURRET_OFFSET_X) * Math.cos(-robotHeading),
                TURRET_OFFSET_Y * Math.cos(-robotHeading) - (TURRET_OFFSET_X) * Math.sin(-robotHeading)
        );
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
    private double getAutoAimAngle()
    {
        double targetTurretAngle;

        // Ensure targetPos is set (e.g., by setAlliance) before calculating.
        // If targetPos is null, we can't calculate a target.
        if (null != offsetTarget && targetPos != null)
        {
            // Vector from Turret offset pos to Goal
            trueTargetVector = offsetTarget.minus(fusedPose.position.plus(getTurretOffsetPosInRobotSpace()));

            // Calculate the absolute field-centric angle to the goal (Radians)
            fieldAngleToGoal = Math.atan2(trueTargetVector.y, trueTargetVector.x);

            // HANDLE IMU WRAPPING:
            // We turn the raw angle into a Rotation2d and subtract our robot heading.
            // This yields the shortest relative distance from robot-front to goal,
            // automatically handling the jump across the +/- 180 degree line.
            double relativeAngleRad = Rotation2d.exp(fieldAngleToGoal).minus(fusedPose.heading);

            // Convert result to Degrees for the Turret Subsystem. Negative because turret rotation is reversed.
            targetTurretAngle = -Math.toDegrees(relativeAngleRad);

            // --- NORMALIZATION LOGIC ---
            // Adjust the calculated target angle to be within the continuous range of the turret
            // and closest to its current position, to avoid unnecessary 360-degree rotations.
            double currentTurret = turret.getCurrentPosition();

            while (180.0 < targetTurretAngle - currentTurret)
            {
                targetTurretAngle -= 360.0;
            }
            while (-180.0 >= targetTurretAngle - currentTurret)
            {
                targetTurretAngle += 360.0;
            }
        }
        else
        {
            // If no pose or target is available, effectively hold current position.
            telemetry.addData("Aiming Mode", "IDLE (No Target Found)");
            return turret.getCurrentPosition();
        }
        return targetTurretAngle;
    }

    /**
     * Commands the turret to hold its current position. This implies manual control.
     */
    public void holdTurretPosition()
    {
        // When this method is called, it signifies a direct manual command, so switch to MANUAL mode.
        if (aimingMode != AimingMode.MANUAL) {
            setAimingMode(AimingMode.MANUAL);
        }
        turret.holdPosition();
    }

    //    void prepShot() {
    //        double distanceInches = getGoalDistance();
    //        double distanceMeters = distanceInches * INCH_TO_METER;
    //        double targetVelocity = flywheel.calculateBallVelocity(distanceMeters, calculateLauncherHeightMeters(distanceMeters), LAUNCH_ANGLE_DEGREES);
    //
    //        flywheel.setTargetRpmFromDistance(distanceInches);
    //    }

    public void prepShot()
    {
        flywheel.setMode(FlywheelController.RunMode.DISTANCE);
    }

    void prepShotLow()
    {
        flywheel.setMode(FlywheelController.RunMode.STATIC);
    }

    public Action prepShotAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                prepShot();
                return true;
            }
        };
    }

    public boolean isAtTargetRpm()
    {
        return flywheel.isAtTargetRpm();
    }

    public Action aimAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                aim();
                return !isAtTarget();
            }
        };
    }

    public Action pointToAction(double angle)
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                aimToAngleInFieldSpace(angle);
                return !isAtTarget();
            }
        };
    }

    public Action stopAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                flywheel.stop();
                return false;
            }
        };
    }

    /**
     * Sets manual power to the turret motor. This implies manual, directional control.
     * @param power The power to apply, from -1.0 to 1.0.
     */
    public void setTurretManualPower(double power)
    {
        // When this method is called, it signifies a direct manual command, so switch to DIRECTIONAL mode.
        if (aimingMode != AimingMode.DIRECTIONAL) {
            setAimingMode(AimingMode.DIRECTIONAL);
        }
        turret.setManualPower(power);
    }

    public void stop()
    {
        flywheel.stop();
    }

    void setAlliance(ThunderBot2025.Alliance_Color color)
    {
        allianceColor = color;
        targetPos = Objects.equals(allianceColor, ThunderBot2025.Alliance_Color.RED) ? targetPosRed : targetPosBlue;
        limelight.setPipeline(Objects.equals(allianceColor, ThunderBot2025.Alliance_Color.RED) ? 2 : 1);
    }


    public double getGoalDistance()
    {
        double distance = trueTargetVector.norm();
        if (null == trueTargetVector || null == offsetTarget)
        {
            distance = 0.0;
        }
        // Use FUSED pose for distance calculation
        if (TELEM)
        {
            telemetry.addData("distance: ", distance);
        }
        return distance;
    }

    boolean isAtTarget()
    {
        return turret.isAtTarget();
    }

}