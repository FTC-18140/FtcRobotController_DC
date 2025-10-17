package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;

@Config
public class ThunderBot2025
{
    public MecanumDrive drive;
    public Intake intake;
    public Indexer indexer;
    public Launcher launcher;
    public LED led;

    private Telemetry telemetry = null;
    public static boolean field_centric = false;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        drive = new MecanumDrive(hwMap, new Pose2d(0,0,0));

        intake = new Intake();
        intake.init(hwMap, telem);

        indexer = new Indexer();
        indexer.init(hwMap, telem);

        launcher = new Launcher();
        launcher.init(hwMap, telem);


        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
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
        public void update(){
            launcher.update();
            led.update(launcher.getAvgtps(), );
        }


}

