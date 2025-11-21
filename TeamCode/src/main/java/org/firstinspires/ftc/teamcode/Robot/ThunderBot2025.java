package org.firstinspires.ftc.teamcode.Robot;

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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.jetbrains.annotations.Nullable;

@Config
public class ThunderBot2025
{
    public MecanumDrive drive;
    public Intake intake;
    public Indexer indexer;
    public LauncherFacade launcher;
    public LED led;
//    public Limelight limelight;
    public enum Alliance_Color{
        RED,
        BLUE
    }
    public Alliance_Color color = Alliance_Color.BLUE;
    private Telemetry telemetry = null;
    public static boolean field_centric = true;
    public static Pose2d starting_position;

    Pose2d TELEOP_START_RED = new Pose2d(-12, -12, 0);
    Pose2d TELEOP_START_BLUE = new Pose2d(-12, 12, 0);



    /**
     * initiolization for ThunderBot2025
     * @param hwMap
     * @param telem
     * @param pose
     */
    public void init(HardwareMap hwMap, Telemetry telem, @Nullable Pose2d pose)
    {
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

        indexer = new Indexer();
        indexer.init(hwMap, telem);

        launcher = new LauncherFacade();
        launcher.init(hwMap, telem);
//        launcher.color = color;
        launcher.setAlliance(color);

        led = new LED();
        led.init(hwMap, telem);
//
//        limelight = new Limelight();
//        limelight.init(hwMap, telem);




        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     *
     * @param alliance a string, supposed to hold the color of the alliance
     */
    public void setColor(Alliance_Color alliance){
        color = alliance;
        launcher.setAlliance(color);

        if(starting_position == null){
            if(alliance == Alliance_Color.RED){
                starting_position = TELEOP_START_RED;
                drive.localizer.setPose(TELEOP_START_RED);
            } else {
                starting_position = TELEOP_START_BLUE;
                drive.localizer.setPose(TELEOP_START_BLUE);
            }
        }

    }

    public void setStartPosForTeleop(Pose2d pos){
        starting_position = pos;
    }

    /**
     * tells you whether you are field centric or not
     * @return the boolean value of whether we are driving in field centric coordinates
     */
    public boolean isFieldCentric(){
        return field_centric;
    }

    /**
     * tells the robot to drive depending on the robot or field centric drive options
     * @param forward
     * @param right
     * @param clockwise
     * @param speed
     * @param p
     */
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

    /**
     * tells the robot how to drive robot centrically
     * @param forward
     * @param right
     * @param clockwise
     * @param speed
     */
    private void robotCentricDrive(double forward, double right, double clockwise, double speed)
    {
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(forward, -right);
        theVector = theVector.times(speed);
        thePose = new PoseVelocity2d(theVector, -clockwise);

        drive.setDrivePowers(thePose);

        telemetry.addData("Odometry X: ", drive.localizer.getPose().position.x);
        telemetry.addData("Odometry Y: ", drive.localizer.getPose().position.y);
    }

    /**
     * tells the robot to drive field centrically
     * @param north
     * @param east
     * @param clockwise
     * @param speed
     * @param p
     */
    private void fieldCentricDrive(double north, double east, double clockwise, double speed, TelemetryPacket p)
    {
        drive.updatePoseEstimate();
        double heading = drive.localizer.getPose().heading.toDouble() - Math.toRadians(90);
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(
                north * Math.cos(-heading) - (-east) * Math.sin(-heading),
                north * Math.sin(-heading) + (-east) * Math.cos(-heading)
        );


        theVector = theVector.times(speed);
        thePose = new PoseVelocity2d(theVector, -clockwise);

        //PoseVelocity2d finalVel = new PoseVelocity2d(new Vector2d(thePose.linearVel.x+0.5*(thePose.linearVel.x-currentVel.linearVel.x),thePose.linearVel.y+0.5*(thePose.linearVel.y-currentVel.linearVel.y)), thePose.angVel+0.5*(thePose.angVel-currentVel.angVel));
        drive.setDrivePowers(thePose);

        }
        /**
         * Calls all of the other update methods
         */
        public void update(){
            drive.updatePoseEstimate();
            drive.localizer.update();
            indexer.update();
            launcher.update( drive.localizer.getPose() );
//            led.update(launcher.avgRpm, launcher.targetRpm);
//            limelight.update();
        }

    /**
     *
     * @return
     */
    public Action updateAction(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    update();
                    return true;
                }
            };
        }




    /**
     * Calls the launcher method lockOn with the degrees that the limelight sees
     */
    public double lockOn(){
//        drive.updatePoseEstimate();
//        return launcher;
        return 0;
    }

    /**
     * launchSeqAction
    * launches 3 artifacts in a given order, i.e, [1,0,2] (fires artifact in slot 1, then slot 0, then slot 2)
    * */
    public Action launchSeqAction(int[] order){
        return new Action() {
            int sequenceStep = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(sequenceStep < order.length) {
                    //cycles to the next artifact
                    indexer.cycleTo(order[sequenceStep]);
                    launchAction();
                    sequenceStep++;
                } else {
                    return false;
                }
                return true;
            }
        };
    }

    /**
     * Tells the launcher to spin up the launch motors with the necessary variables
     */
    public void charge(){
//            launcher.shoot(drive.localizer.getPose(), limelight.distance);

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
                //continuously checks if the turret and flywheel are ready
                return !(launcher.isAtTarget() && launcher.isAtTargetRpm());
            }
        };
    }
        //Actions
        public Action launchAction(){
            return new SequentialAction(
                    //verifies that the turret and flywheel are ready before firing and cycling
                    launchReadyAction(),
                    indexer.flipperUpAction(),
                    new SleepAction(0.15),
                    indexer.flipperDownAndCycleAction(),
                    new SleepAction(0.15)
            );
        }


}

