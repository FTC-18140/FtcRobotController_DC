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
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(-52, 12, Math.toRadians(23));
        Pose2d intakePos = new Pose2d(-34.5, 32, Math.toRadians(90));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        robot.launcher.color = "blue";
        waitForStart();

        robot.setColor("blue");
        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                            new RaceAction(
                                new SequentialAction(
                                    new ParallelAction(
                                            robot.drive.actionBuilder(start)
                                                    .strafeToSplineHeading(new Vector2d(launchPos.position.x, 12), Math.toRadians(23))
                                                    .build()
                                    ),
                                    robot.intake.intakeStartAction(),
                                    //new SleepAction(2),


                                    new SequentialAction(
                                            robot.intake.intakeStopAction(),
                                                robot.launch(),
                                            robot.intake.intakeStartAction(),

                                                robot.indexer.updateAction(),
                                            robot.intake.intakeStopAction(),
                                            robot.launch(),
                                            robot.intake.intakeStartAction(),

                                                robot.indexer.updateAction(),
                                            robot.intake.intakeStopAction(),
                                            robot.launch(),
                                            robot.intake.intakeStartAction()
                                    ),
                                    robot.launcher.stopAction(),
                                    robot.intake.intakeStartAction(),

                                    new ParallelAction(
                                            robot.drive.actionBuilder(launchPos)
                                                    .splineToSplineHeading(intakePos, Math.toRadians(90))
                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, 56), Math.toRadians(90), new TranslationalVelConstraint(6))
                                                    .build(),
                                            new SequentialAction(
                                                    new SleepAction(3),
                                                    robot.indexer.cycleAction(-1),
                                                    new SleepAction(1),
                                                    robot.indexer.cycleAction(-1)
                                            )
                                    ),
                                    new ParallelAction(
                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, 52), Math.toRadians(90)))
                                                .strafeToSplineHeading(launchPos.position, Math.toRadians(23))
                                                .build(),
                                            robot.launcher.turretAimAction(0)
                                    ),
                                    robot.launcher.stopAction(),
                                        new SequentialAction(
                                                robot.intake.intakeStopAction(),
                                                robot.launch(),
                                                robot.intake.intakeStartAction(),

                                                robot.indexer.updateAction(),

                                                robot.intake.intakeStopAction(),
                                                robot.launch(),
                                                robot.intake.intakeStartAction(),

                                                robot.indexer.updateAction(),

                                                robot.intake.intakeStopAction(),
                                                robot.launch(),
                                                robot.intake.intakeStartAction(),

                                                robot.indexer.updateAction(),

                                                robot.intake.intakeStopAction(),
                                                robot.launch(),
                                                robot.intake.intakeStartAction()
                                        )
                                ),
                                    new SleepAction(27)
                            ),
                                robot.intake.intakeStopAction(),
                                robot.drive.actionBuilder(launchPos)
                                        .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(0))
                                        .build(),
                                robot.launcher.turretAimAction(0),
                                robot.launcher.stopAction()

                        ),
                        robot.chargeAction(robot.drive.localizer.getPose(), 30),
                        robot.updateAction()
                )
        );
    }
}
