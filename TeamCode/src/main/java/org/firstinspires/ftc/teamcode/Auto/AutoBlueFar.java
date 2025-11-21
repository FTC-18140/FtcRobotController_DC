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
//        robot.launcher.color = "blue";
        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.BLUE);
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
                            //new SleepAction(2),

                            // Launch Preloads
                            new SequentialAction(
                                robot.launchAction(),

                                robot.launchAction(),

                                robot.launchAction()
                            ),
                            robot.intake.intakeStartAction(),
                            // Grab next 3 artifacts
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
                            // Drive to launch spot
                            new ParallelAction(
                                    robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, 52), Math.toRadians(90)))
                                        .strafeToSplineHeading(launchPos.position, Math.toRadians(23))
                                        .build(),
                                    robot.launcher.pointToAction(0)
                            ),
                            // Launch 2nd set of Artifacts
                            new SequentialAction(
                                    robot.intake.intakeStopAction(),
                                    robot.launchAction(),
                                    robot.intake.intakeStartAction(),

                                    robot.intake.intakeStopAction(),
                                    robot.launchAction(),
                                    robot.intake.intakeStartAction(),

                                    robot.intake.intakeStopAction(),
                                    robot.launchAction(),
                                    robot.intake.intakeStartAction(),

                                    robot.intake.intakeStopAction(),
                                    robot.launchAction(),
                                    robot.intake.intakeStartAction()
                            )
                        ),
                        new SleepAction(27)
                    ),
                    // Park
                    robot.intake.intakeStopAction(),
                    robot.drive.actionBuilder(launchPos)
                            .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(0))
                            .build(),
                    robot.launcher.pointToAction(0),
                    robot.launcher.stopAction()
                ),
                robot.launcher.prepShotAction(),
                robot.updateAction()
            )
        );
    }
}
