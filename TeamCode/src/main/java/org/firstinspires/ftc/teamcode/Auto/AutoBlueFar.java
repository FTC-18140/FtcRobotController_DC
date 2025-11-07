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
public class AutoBlueFar extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_FAR.position, Math.toRadians(-90));
        Pose2d launchPos = new Pose2d(-53, 12, Math.toRadians(24));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        robot.launcher.color = "blue";
        waitForStart();

        robot.setColor("blue");
        Actions.runBlocking(
                new ParallelAction(
                            new SequentialAction(
                                    robot.drive.actionBuilder(start)
                                            .strafeToSplineHeading(new Vector2d(launchPos.position.x, 12), Math.toRadians(24))
                                            .build(),
                                    //new SleepAction(2),

                                    robot.launch(),

                                    robot.indexer.cycleAction(-1),
                                    robot.indexer.updateAction(),

                                    robot.launch(),

                                    robot.indexer.cycleAction(-1),
                                    robot.indexer.updateAction(),

                                    robot.launch(),
                                    robot.indexer.stopAction(),
                                    robot.drive.actionBuilder(launchPos)
                                        .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(0))
                                        .build()
                            ),
                        robot.chargeAction(robot.drive.localizer.getPose(), 20),
                        robot.updateAction(),
                        robot.lockAction()
                )
        );
    }
}
