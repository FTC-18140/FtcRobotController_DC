package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.TelemetryConfig.DEBUG_DRIVE;
import static org.firstinspires.ftc.teamcode.Utilities.LoopTime.LOOP_TIME;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.LoopTime;
import org.jetbrains.annotations.Nullable;
@Config
public class ThunderBot2026
{
    public static final String STARTING_POSE_KEY = "ENDING_POSE_AUTO";
    public MecanumDrive drive = null;
    public Intake intake = null;
    public TransferFacade indexer = null;
//    public LauncherFacade launcher = null;


    public LaunchController launcher = null;
    private LED led = null;
    VoltageSensor voltageSensor = null;
    public LoopTime loopTimer = null;

    // --- AprilTag and Sequence Management ---
    private int latchedObeliskId = -1; // -1 indicates no ID has been officially latched yet.

    public enum Alliance_Color
    {
        RED,
        BLUE
    }

    private ThunderBot2026.Alliance_Color color = ThunderBot2026.Alliance_Color.BLUE;
    public TransferFacade.BallState lastBallState = null;
    private Telemetry telemetry = null;
    public static boolean field_centric = false;
    public static final double MIN_SPEED = 0.3;
    public static final double DEFAULT_SPEED = 0.7;
    public static final double MAX_SPEED = 1.0;
    private double speed = DEFAULT_SPEED;
    public static Pose2d starting_position = null;
    public static double robot_width = 14;
    double halfWidth = robot_width / 2;
    /*         _left__
              [0]   [1]
              |       | front
              [2]   [3]
     */
    double[][] starting_corners = {
            {halfWidth, -halfWidth}, {halfWidth, halfWidth},
            {-halfWidth, -halfWidth}, {-halfWidth, halfWidth}
    };
    double[][] corners = {
            {halfWidth, -halfWidth}, {halfWidth, -halfWidth},
            {-halfWidth, -halfWidth}, {-halfWidth, halfWidth}
    };
    boolean inZone = false;
    private static String STARTING_POSE = STARTING_POSE_KEY;
    public ElapsedTime runtime = new ElapsedTime();
    private Pose2d TELEOP_CORNER_RED = new Pose2d(-63, 60, 0);
    private Pose2d TELEOP_CORNER_BLUE = new Pose2d(-63, -60, 0);

    ElapsedTime updateTimer = new ElapsedTime(ElapsedTime.SECOND_IN_NANO);

    public void init(HardwareMap hwMap, Telemetry telem, @Nullable Pose2d pose)
    {
//        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());

        loopTimer = new LoopTime();
        loopTimer.init();
        telemetry = telem;
        starting_position = (Pose2d) OpMode.blackboard.getOrDefault(STARTING_POSE, null);
        if (pose == null)
        {
            if (starting_position == null)
            {
                pose = new Pose2d(0, 0, 0);
            }
            else
            {
                pose = starting_position;
            }
        }
        // Get the voltage sensor with the lowest voltage.
        double minVoltage = Double.POSITIVE_INFINITY;
        VoltageSensor bestSensor = null;

        for (VoltageSensor sensor : hwMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();

            if (voltage > 0 && voltage < minVoltage)
            {
                minVoltage = voltage;
                bestSensor = sensor;
            }
        }
        if (bestSensor == null)
        {
            throw new RuntimeException("No valid voltage sensor found");
        }
        voltageSensor = bestSensor;

        if ( getBatteryVoltage() < 13.0 )
        {
            telemetry.addData("Replace Battery please, I Beg you", 0);
        }

        drive = new MecanumDrive(hwMap, pose);

        intake = new Intake();
        intake.init(hwMap, telemetry);

        indexer = new TransferFacade();
        indexer.init(hwMap, telemetry);
        indexer.intakeStop();

        launcher = new LaunchController();
        launcher.init(hwMap, telemetry, pose);

        led = new LED();
        led.init(hwMap, telemetry, Flywheel.RPM_LOWER_BOUND, Flywheel.RPM_UPPER_BOUND);

        runtime.reset();
    }

    public void update()
    {
        double seconds = runtime.seconds();
        loopTimer.update();

        updateTimer.reset();
        PoseVelocity2d robotPoseVel = drive.updatePoseEstimate();
        double poseUpdateTime = updateTimer.seconds();

        if ( DEBUG_DRIVE)
        {
            addTelemetry();
        }

        updateTimer.reset();
        launcher.update(drive.localizer.getPose(), robotPoseVel );
        //, getBatteryVoltage(), indexer.getCurrentState() == TransferFacade.State.LAUNCHING);
        double launcherUpdateTime = updateTimer.seconds();

        boolean atTargetRpm = launcher.isAtTargetRpm();
        boolean atTarget = launcher.isAtTarget();

        inZone = inLaunchZone();

        updateTimer.reset();
        indexer.update(atTargetRpm);
        double indexerUpdateTime = updateTimer.seconds();

        updateTimer.reset();
        intake.update(!(indexer.getCurrentState() == TransferFacade.State.IDLE));
        double intakeUpdateTime = updateTimer.seconds();

        lastBallState = indexer.getLaunchColor();
        double flywheelTargetRpm = launcher.getFlywheelTargetRpm();
        double flywheelRpm = launcher.getLowerFlywheelRpm();

        boolean isIndexerFull = indexer.indexerIsFull();
        boolean isIntakeFull = indexer.ballInIntake();
        TransferFacade.State state = indexer.getCurrentState();

        updateTimer.reset();
        led.update(inZone, indexer.isOverridden(), seconds, lastBallState, isIndexerFull, isIntakeFull, state);
        double ledUpdateTime = updateTimer.seconds();

        if (indexer.getCurrentState() == TransferFacade.State.MOVING)
        {
            intake.setSlowSpeed();
        }
        else
        {
            intake.setNormalSpeed();
//            intake.intake();
        }

        if (indexer.tooManyArtifacts())
        {
            intake.spit();
        }

        telemetry.addData("Loop Time", LOOP_TIME);
//        telemetry.addData("Pose Update", poseUpdateTime);
//        telemetry.addData("Launcher Update", launcherUpdateTime);
//        telemetry.addData("Indexer Update", indexerUpdateTime);
//        telemetry.addData( "LED Update", ledUpdateTime);
//        telemetry.addData("Intake Update", intakeUpdateTime);
        telemetry.update();
    }

    private void addTelemetry()
    {
        telemetry.addData("Time since start", runtime.seconds());
        telemetry.addData("Battery Voltage", getBatteryVoltage());
        telemetry.addData("Total Motor Current Draw", getTotalMotorCurrentDraw());

        telemetry.addData("position X: ", drive.localizer.getPose().position.x);
        telemetry.addData("position Y: ", drive.localizer.getPose().position.y);
        telemetry.addData("heading: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

        telemetry.addData("Flywheel RPM ", launcher.getLowerFlywheelRpm());
        telemetry.addData("Flywheel Target ", launcher.getFlywheelTargetRpm());

        telemetry.addData("Upper Flywheel RPM ", launcher.getUpperFlywheelRpm());
        telemetry.addData("Upper Flywheel Target ", launcher.getUpperFlywheelTargetRpm());
        telemetry.addData("Turret angle:", launcher.getTurretAngle());
    }

    public void setColor(ThunderBot2026.Alliance_Color alliance)
    {
        color = alliance;
        ThunderBot2025.Alliance_Color color_2025;
        switch (alliance)
        {
            case RED:
                color_2025 = ThunderBot2025.Alliance_Color.RED;
                break;
            case BLUE:
                color_2025 = ThunderBot2025.Alliance_Color.BLUE;
                break;
            default:
                color_2025 = ThunderBot2025.Alliance_Color.BLUE;
                break;
        }
        launcher.setAlliance(color_2025);
    }


    public double getBatteryVoltage()
    {
        if ( voltageSensor != null )
        {
            return voltageSensor.getVoltage();
        }
        else
        {
            return 0.0;
        }
    }

    public double getTotalMotorCurrentDraw()
    {
        return launcher.getTotalCurrentDraw() + intake.getTotalCurrentDraw() + drive.getTotalCurrentDraw();
    }

    /**
     * This method is now responsible for both latching the Obelisk ID and planning the sequence.
     * The first time it is called with a valid AprilTag in view, it "latches" that ID.
     * Every subsequent call will use the latched ID, ignoring any new tags the robot might see.
     */
    public boolean registerObeliskID()
    {

//        // Step 1: Latch the official ID if we haven't already.
//
//        launcher.setPipeline((ThunderBot2026.Alliance_Color.BLUE == color) ? 0 : 3);
//        int currentId = launcher.getDetectedAprilTagId();
//        if (-1 != currentId)
//        {
//            latchedObeliskId = currentId;
//            telemetry.addData("Obelisk ID Latched: ", latchedObeliskId);
//        }
//
//        // Step 2: Plan the sequence using the latched ID.
//        // This will only proceed if an ID has been successfully latched.
//        if (-1 != latchedObeliskId)
//        {
//            telemetry.addData("Sequence Planned:", indexer.planShotSequence(latchedObeliskId));
//            return true;
//        }
        return false;
    }

    public void drive(double forward, double right, double clockwise, double inSpeed)
    {
        speed = inSpeed;
        if (field_centric)
        {
            fieldCentricDrive(forward, right, clockwise, speed);
        }
        else
        {
            robotCentricDrive(forward, right, clockwise, speed);
        }
    }

    private void robotCentricDrive(double forward, double right, double clockwise, double speed)
    {
        PoseVelocity2d thePose = new PoseVelocity2d(new Vector2d(-forward, right).times(speed), -clockwise);
        drive.setDrivePowers(thePose);
    }

    private void fieldCentricDrive(double north, double east, double clockwise, double speed)
    {
//        drive.updatePoseEstimate();
        double heading = drive.localizer.getPose().heading.toDouble() - Math.toRadians(90);
        Vector2d theVector = new Vector2d(
                north * Math.cos(-heading) - (-east) * Math.sin(-heading),
                north * Math.sin(-heading) + (-east) * Math.cos(-heading)
        );

        theVector = theVector.times(speed);
        PoseVelocity2d thePose = new PoseVelocity2d(theVector, -clockwise * speed);
        drive.setDrivePowers(thePose);
    }

    public boolean inLaunchZone()
    {
        double botX = drive.localizer.getPose().position.x;
        double botY = drive.localizer.getPose().position.y;
        double h = drive.localizer.getPose().heading.toDouble();

        for (int i = 0; i < corners.length; i++)
        {
            double x = starting_corners[i][0];
            double y = starting_corners[i][1];
            corners[i][0] = y * Math.sin(-h) + (x) * Math.cos(-h);
            corners[i][1] = y * Math.cos(-h) - (x) * Math.sin(-h);
        }
        if (botY < halfWidth && botY > -halfWidth)
        {
            if (0 < botX + halfWidth || -48 > botX - halfWidth)
            {
                return true;
            }
        }
        for (int i = 0; i < corners.length; i++)
        {
            double x = corners[i][0] + botX;
            double y = corners[i][1] + botY;

            if (0 < x)
            {
                if (y < x && y > -x)
                {
                    return true;
                }
            }
            else
            {
                if (y < (-x - 48) && y > (x + 48))
                {
                    return true;
                }
            }
        }

        return false;
    }

    public Action waitForTime(double time)
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                return runtime.seconds() < time;
            }
        };
    }

    public void intakeStart()
    {
        intake.intake();
        indexer.intakeStart();
    }

    public void intakeStop()
    {
        intake.stop();
        indexer.intakeStop();
    }

    public Action intakeStartAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                intakeStart();
                return false;
            }
        };
    }

    public Action intakeStopAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                intakeStop();
                return false;
            }
        };
    }

    public void charge()
    {
        launcher.prepShot();
    }

    public void chargeLow()
    {
        launcher.prepShotLow();
    }

    public void chargeStop()
    {
        launcher.stopFlywheel();
    }

    public boolean launch()
    {
        if (launcher.isAtTargetRpm() && inZone)
        {
            indexer.launch();
            return true;
        }
        return false;
    }

    public boolean launchAll()
    {
        if ((launcher.isAtTargetRpm() && inZone) || indexer.isOverridden())
        {
            indexer.launchAll();
            return true;
        }
        return false;
    }

//    public void flipperUp() {
//        indexer.flipOverride(true);
//    }
//
//    public void flipperDown() {
//        indexer.flipOverride(false);
//    }


    public Action updateAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                update();
                telemetry.update();
                return true;
            }
        };
    }


    public void resetHeadingAndPosition()
    {

        drive.localizer.setPose((ThunderBot2026.Alliance_Color.RED == color) ? TELEOP_CORNER_RED : TELEOP_CORNER_BLUE);
    }

    public boolean resetTurret()
    {
//        if (launcher.setTurretOffset())
//        {
//            led.setLauncherLedToColor(LED.Colors.GREEN);
//            return true;
//        }
//        else
//        {
//            led.setLauncherLedToColor(LED.Colors.BLUE);
//            return false;
//        }
        return true;
    }

    public Action aimAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                launcher.autoAim();
                return true;
            }
        };
    }

    public Action holdTurretAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                launcher.holdTurretPosition();
                return true;
            }
        };
    }

    // --- Start of New Intelligent Actions ---

    public Action spamAction()
    {
        return new SequentialAction(
                launchAction(),
                launchAction(),
                launchAction(),
                new SleepAction(0.2)
        );
    }

    public Action startSequenceAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                return !indexer.prepSequence();
            }
        };
    }

    public Action sortAndLaunchAction()
    {
        return new SequentialAction(
                startSequenceAction(),
                launchAction()
        );
    }

    public Action cancelSequenceAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                indexer.cancelSequence();
                return false;
            }
        };
    }

    public Action waitForSequenceEndAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                return indexer.isInSequence();
            }
        };
    }

//    public Action waitForBallAndCycleAction() {
//        return new Action() {
//            private boolean hasStarted = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//
//                if (indexer.ballInIndexer() && indexer.isAtTarget() && !hasStarted) {
//                    hasStarted = !indexer.readyNextIntakeSlot(IndexerFacade.BallState.VACANT); // End this action, the cycle command has been sent.
//                } else if (hasStarted) {
//                    return IndexerFacade.State.SELECTING_BALL != indexer.getCurrentState();
//                }
//                return true; // Continue waiting for a ball.
//            }
//        };
//    }
//
//    public Action waitForBallAction() {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                return indexer.ballInIndexer();
//            }
//        };
//    }

    // --- Deprecated and Re-implemented Actions ---

    public Action indexerFullAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                return !indexer.indexerIsFull();
            }
        };
    }

//    public Action indexerIsAtTargetAction() {
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                return !indexer.isAtTarget();
//            }
//        };
//    }

    public Action launchAction()
    {
        return new SequentialAction(
                new Action()
                {
                    boolean started = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket)
                    {
                        if (!started)
                        {
                            if (launchAll())
                            {
                                intakeStop();
                                started = true;
                            }
                            return true;
                        }
                        else
                        {
//                            return !indexer.isNearSlot();
                            return indexer.getCurrentState() == TransferFacade.State.LAUNCHING;
                        }
                    }
                }
        );
    }

    /**
     * Returns a Road Runner Action that plans the shot sequence based on the last detected Obelisk ID.
     * This is useful for re-planning the sequence after intaking new artifacts.
     *
     * @return An Action that can be used in a sequence.
     */
    public Action planSequenceAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                indexer.planShotSequence(latchedObeliskId);
                return !indexer.isInSequence(); // This is a one-shot action.
            }
        };
    }
}
