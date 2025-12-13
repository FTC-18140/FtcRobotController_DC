package org.firstinspires.ftc.teamcode.Robot;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.DataLoggable;
import org.firstinspires.ftc.teamcode.Utilities.DataLogger;
import org.jetbrains.annotations.Nullable;

@Config
public class ThunderBot2025 implements DataLoggable
{
    public MecanumDrive drive;
    public Intake intake;
    public IndexerFacade indexer;
    public LauncherFacade launcher;
    public LED led;

    // --- AprilTag and Sequence Management ---
    private int latchedObeliskId = -1; // -1 indicates no ID has been officially latched yet.

    public enum Alliance_Color{
        RED,
        BLUE
    }
    public Alliance_Color color = Alliance_Color.BLUE;
    private Telemetry telemetry = null;
    public static boolean field_centric = true;
    public static Pose2d starting_position;
    public static String STARTING_POSITION = "ENDING_POSITION_AUTO";

    Pose2d TELEOP_START_RED = new Pose2d(-12, -12, 0);
    Pose2d TELEOP_START_BLUE = new Pose2d(-12, 12, 0);

    public void init(HardwareMap hwMap, Telemetry telem, @Nullable Pose2d pose)
    {
        starting_position = (Pose2d) blackboard.getOrDefault(STARTING_POSITION, null);
        if(pose == null){
            if(starting_position == null) {
                pose = new Pose2d(0,0,0);
            } else {
                pose = starting_position;
            }
        }
        drive = new MecanumDrive(hwMap, pose);

        intake = new Intake();
        intake.init(hwMap, telem);

        indexer = new IndexerFacade();
        indexer.init(hwMap, telem);

        launcher = new LauncherFacade();
        launcher.init(hwMap, telem);
        launcher.setAlliance(color);

        led = new LED();
        led.init(hwMap, telem);

        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void setColor(Alliance_Color alliance){
        color = alliance;
        launcher.setAlliance(color);

//        if(starting_position == null){
//            if(alliance == Alliance_Color.RED){
//                starting_position = TELEOP_START_RED;
//                drive.localizer.setPose(TELEOP_START_RED);
//            } else {
//                starting_position = TELEOP_START_BLUE;
//                drive.localizer.setPose(TELEOP_START_BLUE);
//            }
//        }
    }

    /**
     * This method is now responsible for both latching the Obelisk ID and planning the sequence.
     * The first time it is called with a valid AprilTag in view, it "latches" that ID.
     * Every subsequent call will use the latched ID, ignoring any new tags the robot might see.
     */
    public void registerObeliskID(){
        // Step 1: Latch the official ID if we haven't already.
        if (latchedObeliskId == -1) {
            int currentId = launcher.getDetectedAprilTagId();
            if (currentId != -1) {
                latchedObeliskId = currentId;
                telemetry.addData("Obelisk ID Latched: ", latchedObeliskId);
            }
        }

        // Step 2: Plan the sequence using the latched ID.
        // This will only proceed if an ID has been successfully latched.
        if (latchedObeliskId != -1) {
            indexer.planShotSequence(latchedObeliskId);
        }
    }

    public void setStartPosForTeleop(Pose2d pos){
        starting_position = pos;
    }

    public boolean isFieldCentric(){
        return field_centric;
    }

    public void drive( double forward, double right, double clockwise, double speed, TelemetryPacket p)
    {
        if (field_centric)
        {
            fieldCentricDrive(forward, right, clockwise, speed, p);
        }
        else
        {
            robotCentricDrive(forward, right, clockwise, speed);
        }
    }

    private void robotCentricDrive(double forward, double right, double clockwise, double speed)
    {
        PoseVelocity2d thePose = new PoseVelocity2d(new Vector2d(forward, -right).times(speed), -clockwise);
        drive.setDrivePowers(thePose);
        telemetry.addData("Odometry X: ", drive.localizer.getPose().position.x);
        telemetry.addData("Odometry Y: ", drive.localizer.getPose().position.y);
    }

    private void fieldCentricDrive(double north, double east, double clockwise, double speed, TelemetryPacket p)
    {
        drive.updatePoseEstimate();
        double heading = drive.localizer.getPose().heading.toDouble() - Math.toRadians(90);
        Vector2d theVector = new Vector2d(
                north * Math.cos(-heading) - (-east) * Math.sin(-heading),
                north * Math.sin(-heading) + (-east) * Math.cos(-heading)
        );

        theVector = theVector.times(speed);
        PoseVelocity2d thePose = new PoseVelocity2d(theVector, -clockwise * speed);
        drive.setDrivePowers(thePose);
    }

    public void update(){
        drive.updatePoseEstimate();
        drive.localizer.update();
        indexer.update();
        launcher.update(this.drive.localizer.getPose());
    }

    public void charge() {
        launcher.prepShot();
    }

    public Action updateAction(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    update();
                    telemetry.update();
                    return true;
                }
            };
        }

    public double lockOn(){
        return 0;
    }

    public Action chargeAction(Pose2d pose, double duration){
        return new Action() {
            ElapsedTime chargeTimer = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    chargeTimer.reset();
                    started = true;
                }
                launcher.prepShot();
                telemetry.addData("time: ", chargeTimer.seconds());

                if (chargeTimer.seconds() > duration){
                    launcher.stop();
                    return false;
                }
                return true;
            }
        };
    }

    public Action aimAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                launcher.aim();
                return true;
            }
        };
    }

    public Action launchReadyAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !(launcher.isAtTarget() && launcher.isAtTargetRpm() && indexer.isAtTarget() && (indexer.getState() == IndexerFacade.State.AWAITING_FLIP || indexer.getState() == IndexerFacade.State.IDLE));
            }
        };
    }
    
    // --- Start of New Intelligent Actions ---
    
    public Action seekToSlotAction(int slot) {
        return new Action() {
            private boolean hasStarted = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!hasStarted) {
                    indexer.selectSlot(slot);
                    hasStarted = true;
                }
                return !indexer.isDone();
            }
        };
    }
    
    public Action waitForBallAndCycleAction() {
        return new Action() {
            private boolean hasStarted = false;
            private int slotToWatch;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!hasStarted) {
                    slotToWatch = 0;
                    hasStarted = true;
                }

                // Condition to exit: if the slot we were watching is no longer vacant.
                if (indexer.getBallState(slotToWatch) != IndexerFacade.BallState.VACANT) {

                    return !indexer.selectNextSlot(IndexerFacade.BallState.VACANT); // End this action, the cycle command has been sent.
                }
                return true; // Continue waiting for a ball.
            }
        };
    }
    public Action waitForBallAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return indexer.getBallState(0) == IndexerFacade.BallState.VACANT;
            }
        };
    }

    // --- Deprecated and Re-implemented Actions ---

    public Action cycleAction(int ignored) {
        return seekToSlotAction(0); // Default to something safe, but shouldn't be used
    }

    public Action indexerFullAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return !indexer.indexerIsFull();
            }
        };
    }
    public Action launchAction(){
        return new SequentialAction(
                launchReadyAction(),
                new Action() { 
                    private boolean hasStarted = false;
                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!hasStarted) {
                           hasStarted = indexer.flip();
                           telemetry.addData("awaiting flip", indexer.getState());
                        } else {
                            telemetry.addData("cycling", 0);
                            return !indexer.cycle(1);
                        }
                        return true;
                    }
                },
                new SleepAction(0.45)
        );
    }

    /**
     * Returns a Road Runner Action that plans the shot sequence based on the last detected Obelisk ID.
     * This is useful for re-planning the sequence after intaking new artifacts.
     * @return An Action that can be used in a sequence.
     */
    public Action planSequenceAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                registerObeliskID();
                return false; // This is a one-shot action.
            }
        };
    }

    @Override
    public void logData(DataLogger logger) {
        launcher.logData(logger);
        Pose2d pose = drive.localizer.getPose();
        logger.addField(pose.position.x);
        logger.addField(pose.position.y);
        logger.addField(pose.heading.toDouble());
    }
}
