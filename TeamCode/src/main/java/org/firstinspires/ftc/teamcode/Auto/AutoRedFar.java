package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous
public class AutoRedFar extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(90));
        Pose2d launchPos = new Pose2d(-52, -12, Math.toRadians(-24));


        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        robot.launcher.color = "red";
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        new ParallelAction(
                            new SequentialAction(
                                    robot.drive.actionBuilder(start)
                                            .strafeToSplineHeading(new Vector2d(launchPos.position.x, -12), Math.toRadians(-24))
                                            .build(),
                                    //new SleepAction(2),

                                    robot.launcher.waitForCharge(robot.drive.localizer.getPose()),
                                    robot.indexer.fliperAction(0.65),
                                    new SleepAction(0.5),
                                    robot.indexer.fliperAction(0.25),
                                    new SleepAction(0.1),

                                    robot.indexer.cycleAction(-1),
                                    robot.indexer.updateAction(),
                                    robot.indexer.stopAction(),

                                    robot.launcher.waitForCharge(robot.drive.localizer.getPose()),
                                    robot.indexer.fliperAction(0.65),
                                    new SleepAction(0.75),
                                    robot.indexer.fliperAction(0.25),

                                    robot.indexer.cycleAction(-1),
                                    robot.indexer.updateAction(),

                                    robot.launcher.waitForCharge(robot.drive.localizer.getPose()),
                                    robot.indexer.fliperAction(0.65),
                                    new SleepAction(0.25),
                                    robot.indexer.fliperAction(0.25),

                                    robot.drive.actionBuilder(launchPos)
                                            .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(0))
                                            .build()
                            ),
                                robot.launcher.chargeAction(robot.drive.localizer.getPose(), 25)
                        ),
                        robot.launcher.updateAction()
                )
        );
    }
}
