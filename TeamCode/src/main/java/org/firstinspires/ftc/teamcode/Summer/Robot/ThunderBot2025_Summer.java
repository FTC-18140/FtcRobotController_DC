package org.firstinspires.ftc.teamcode.Summer.Robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ThunderBot2025_Summer
{
    public MecanumDrive_PinPoint drive;

    private Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telem)
    {
        drive = new MecanumDrive_PinPoint(hwMap, new Pose2d(0,0,0));

        telemetry = telem;
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
    public void fieldCentricDrive(double north, double east, double clockwise, double speed)
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
    }
}

