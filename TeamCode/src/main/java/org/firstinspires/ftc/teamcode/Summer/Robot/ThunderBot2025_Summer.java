package org.firstinspires.ftc.teamcode.Summer.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ThunderBot2025_Summer
{
    public MecanumDrive_PinPoint drive;

    private Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        drive = new MecanumDrive_PinPoint(hwMap, new Pose2d(0,0,0));

        telemetry = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void robotCentricDrive(double forward, double right, double clockwise, double speed)
    {
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(forward, -right);
        theVector = theVector.times(speed);
        thePose = new PoseVelocity2d(theVector, -clockwise);


        PoseVelocity2d currentVel = drive.updatePoseEstimate();
        //PoseVelocity2d finalVel = new PoseVelocity2d(new Vector2d(thePose.linearVel.x+0.5*(thePose.linearVel.x-currentVel.linearVel.x),thePose.linearVel.y+0.5*(thePose.linearVel.y-currentVel.linearVel.y)), thePose.angVel+0.5*(thePose.angVel-currentVel.angVel));
        drive.setDrivePowers(thePose);


        telemetry.addData("Odometry X: ", drive.pose.position.x);
        telemetry.addData("Odometry Y: ", drive.pose.position.y);
    }
    public void fieldCentricDrive(double north, double east, double clockwise, double speed, TelemetryPacket p)
    {
        drive.updatePoseEstimate();
        double heading = drive.pose.heading.toDouble();
        PoseVelocity2d thePose;
        Vector2d theVector;
        theVector = new Vector2d(
                north * Math.cos(-heading) - east * Math.sin(-heading),
                north * Math.sin(-heading) + east * Math.cos(-heading)
        );


        theVector = theVector.times(speed);
        thePose = new PoseVelocity2d(theVector, -clockwise);

        //PoseVelocity2d finalVel = new PoseVelocity2d(new Vector2d(thePose.linearVel.x+0.5*(thePose.linearVel.x-currentVel.linearVel.x),thePose.linearVel.y+0.5*(thePose.linearVel.y-currentVel.linearVel.y)), thePose.angVel+0.5*(thePose.angVel-currentVel.angVel));
        drive.setDrivePowers(thePose);

        Canvas c = p.fieldOverlay();

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, drive.pose);

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, new Pose2d(theVector, -clockwise));

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokeLine(drive.pose.position.x, drive.pose.position.y, drive.pose.position.x + theVector.x, drive.pose.position.y + theVector.y);
    }
}

