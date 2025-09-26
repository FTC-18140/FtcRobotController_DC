package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drives.MecanumDrive;
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(-24, 24), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-12, 12, 0), 0)
                        .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(90)), Math.toRadians(-90))
                .build()
        );

    }
}
