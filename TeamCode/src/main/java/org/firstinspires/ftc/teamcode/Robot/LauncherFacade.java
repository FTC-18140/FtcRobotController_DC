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
     *
     * @param currentOdoPose     The raw pose from RoadRunner drive.getPoseEstimate()
     * @param currentOdoVelocity
     */
    public void update(Pose2d currentOdoPose, PoseVelocity2d currentOdoVelocity, double voltage, boolean launching)
    {
        if (null == lastOdoPose)
        {
            lastOdoPose = currentOdoPose;
            return;
        }

        double update_rate_seconds = (System.currentTimeMillis() - last_time_ms) / 1000;
        lastOdoPose = currentOdoPose;

        fusedPose = currentOdoPose;
        inertiaOffset = currentOdoVelocity.linearVel.times(INERTIA_FACTOR);
        offsetTarget = targetPos.minus(inertiaOffset);

        getAutoAimAngle();
        double distanceToGoal = getGoalDistance();
        turret.update(fusedPose, currentOdoVelocity, offsetTarget, launching);
        flywheel.update(currentOdoVelocity, Math.toDegrees(fieldAngleToGoal), voltage, distanceToGoal);
        last_time_ms = System.currentTimeMillis();

        if (TELEM)
        {
            telemetry.addData("update rate (seconds): ", update_rate_seconds);
        }
    }

    public void setAimingMode(AimingMode mode)
    {
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

    public void aim()
    {
        double instantTarget = getAutoAimAngle();

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
            smoothedTurretAngle += (delta * LPF_BETA);
        }
        double baseAngle = turret.applyHardwareConstraints(smoothedTurretAngle);

        // --- FINAL COMMAND ---

        // Command the turret subsystem to the calculated angle.
        turret.seekToAngle(baseAngle);

        double currentPosition = turret.getCurrentPosition();
        if (TELEM)
        {
            telemetry.addData("Turret Current", currentPosition);
            telemetry.addData("Turret Target", baseAngle);
        }
    }

    boolean setTurretOffset()
    {
        // Calculate the vector (x, y) pointing from the robot to the goal
        boolean returnValue = false;
        setAimingMode(AimingMode.DIRECTIONAL);
        if (turret.isHomed())
        {
            double offset = getTurretAngleRaw();
            turret.setOffsetAngle(offset);
            returnValue = true;
        }
        return returnValue;
    }

    public void aimToAngleInFieldSpace(double angle)
    {
        double robotHeadingDouble = fusedPose.heading.toDouble();
        double robotHeadingDegrees = Math.toDegrees(robotHeadingDouble);
        double targetAngle = turret.applyHardwareConstraints(robotHeadingDegrees - angle);
        turret.seekToAngle(targetAngle);
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

        // --- 2. SENSOR PRIORITY: ODOMETRY ---
        // Fallback to Odometry if the Limelight is blocked or target is out of view.
        // We calculate the vector from our fused robot position to the field goal position.
        if (null != offsetTarget)
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

            // Convert result to Degrees for the Turret Subsystem
            targetTurretAngle = -Math.toDegrees(relativeAngleRad);

            // --- NORMALIZATION LOGIC ---
            // Since the turret can go up to 225, a result of -170 (from the RR math)
            // is the same as 190. If the turret is currently at 180, we want to
            // go to 190, NOT -170.
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

        // --- 3. FALLBACK: IDLE ---
        // If no pose or target is available, hold current position to prevent erratic movement.
        else
        {
            telemetry.addData("Aiming Mode", "IDLE (No Target Found)");
            return turret.getCurrentPosition();
        }
        return targetTurretAngle;
    }

    public void holdTurretPosition()
    {
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
//        flywheel.setTargetRpm(Flywheel.STATIC_RPM);
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
                turret.seekToAngle(angle);
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

    public void setTurretManualPower(double power)
    {
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
