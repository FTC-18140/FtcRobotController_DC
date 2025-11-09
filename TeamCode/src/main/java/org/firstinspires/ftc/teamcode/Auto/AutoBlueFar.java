package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
        Pose2d intakePos = new Pose2d(-32, 48, Math.toRadians(90));

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
                                    robot.intake.intakeStartAction(),
                                    //new SleepAction(2),

                                    new RaceAction(
                                            new SequentialAction(
                                                robot.launch(),

                                                robot.indexer.cycleAction(-1),
                                                robot.indexer.updateAction(),

                                                robot.launch(),

                                                robot.indexer.cycleAction(-1),
                                                robot.indexer.updateAction(),

                                                robot.launch()
                                            ),
                                            new SleepAction(10)
                                    ),

                                    new ParallelAction(
                                            robot.drive.actionBuilder(launchPos)
                                                    .splineToSplineHeading(intakePos, Math.toRadians(90))
                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, 52), Math.toRadians(90), new TranslationalVelConstraint(12))
                                                    .build(),
                                            new SequentialAction(
                                                    new SleepAction(2),
                                                    robot.indexer.cycleAction(-1),
                                                    new SleepAction(1.5),
                                                    robot.indexer.cycleAction(-1),
                                                    new SleepAction(1.5),
                                                    robot.indexer.cycleAction(-1)
                                            )
                                    ),
                                    robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, 52), Math.toRadians(90)))
                                            .strafeToSplineHeading(launchPos.position, Math.toRadians(24))
                                            .build(),

                                    new RaceAction(
                                            new SequentialAction(
                                                    robot.launch(),

                                                    robot.indexer.cycleAction(-1),
                                                    robot.indexer.updateAction(),

                                                    robot.launch(),

                                                    robot.indexer.cycleAction(-1),
                                                    robot.indexer.updateAction(),

                                                    robot.launch()
                                            ),
                                            new SleepAction(10)
                                    ),

                                    robot.intake.intakeStopAction(),
                                    robot.drive.actionBuilder(launchPos)
                                        .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(0))
                                        .build()
                            ),
                        robot.chargeAction(robot.drive.localizer.getPose(), 25),
                        robot.updateAction(),
                        new SequentialAction(
                                new SleepAction(1),
                                robot.lockAction()
                        )
                )
        );
    }
}
