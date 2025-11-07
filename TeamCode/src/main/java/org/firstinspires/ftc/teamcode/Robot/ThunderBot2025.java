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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
import org.jetbrains.annotations.Nullable;

import java.util.Objects;

@Config
public class ThunderBot2025
{
    public MecanumDrive drive;
    public Intake intake;
    public Indexer indexer;
    public Launcher launcher;
    public LED led;
    public Limelight limelight;

    public String color = "blue";
    private Telemetry telemetry = null;
    public static boolean field_centric = false;

    public void init(HardwareMap hwMap, Telemetry telem, @Nullable Pose2d pose)
    {
        if(pose == null){
            pose = new Pose2d(0,0,0);
        }
        drive = new MecanumDrive(hwMap, pose);

        intake = new Intake();
        intake.init(hwMap, telem);

        indexer = new Indexer();
        indexer.init(hwMap, telem);

        launcher = new Launcher();
        launcher.init(hwMap, telem);
        launcher.color = color;

        led = new LED();
        led.init(hwMap, telem);

        limelight = new Limelight();
        limelight.init(hwMap, telem);




        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void setColor(String col){
        color = col;
        if (Objects.equals(color, "blue")) {
            limelight.SetPipeline(1);
            launcher.turret_target_pos = 0.5;
        }else {
            limelight.SetPipeline(2);
            launcher.turret_target_pos = 0.5;
        }
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
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(forward, -right);
        theVector = theVector.times(speed);
        thePose = new PoseVelocity2d(theVector, -clockwise);

        drive.setDrivePowers(thePose);

        telemetry.addData("Odometry X: ", drive.localizer.getPose().position.x);
        telemetry.addData("Odometry Y: ", drive.localizer.getPose().position.y);
    }
    private void fieldCentricDrive(double north, double east, double clockwise, double speed, TelemetryPacket p)
    {
        drive.updatePoseEstimate();
        double heading = drive.localizer.getPose().heading.toDouble();
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
            indexer.update();
            launcher.update();
            led.update(launcher.avgRpm, launcher.power);
            limelight.update();
            limelight.xdegrees();
        }


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
     * turns the turret to point down the field (hopefully)
     * reads the id
     * @param pipeline the pipeline you want it to read
     * @return returns the id
     */
    public int lookForId(int pipeline, String alliance){
        if (Objects.equals(alliance, "blue")) {
            launcher.turnToPosition(1);
        } else {
            launcher.turnToPosition(-1);
        }
            limelight.SetPipeline(pipeline);
            return limelight.id();
    }

    /**
     * Calls the launcher method lockOn with the degrees that the limelight sees
     */
    public void lockOn(){
            launcher.lockOn(limelight.xdegrees());

        }
        public void charge(){
            launcher.shoot(drive.localizer.getPose(), limelight.distance);
        }
    public Action chargeAction(Pose2d pose, double duration){
        return new Action() {
            ElapsedTime counter = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    counter.reset();
                    started = true;
                }
                charge();

                telemetry.addData("time: ", counter.seconds());

                if (counter.seconds() > duration){
                    launcher.stop();
                    return false;
                }
                return true;
            }
        };
    }
        public Action lockAction(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    lockOn();
                    return true;
                }
            };
        }
        //Actions
        public Action launch(){
            return new SequentialAction(
                    launcher.waitForCharge(),
                    indexer.flipperUpAction(),
                    new SleepAction(0.25),
                    indexer.flipperDownAction()
            );
        }


}

