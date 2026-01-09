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
public class AutoRedFar_WAIT extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.FAR_LAUNCH_ZONE_RED.position, Math.toRadians(-23));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_BASE_RED.position, Math.toRadians(90));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);
        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.RED);
        Actions.runBlocking(
                new ParallelAction(
                        robot.updateAction(),
                        robot.aimAction(),

                        new SequentialAction(
                                new SleepAction(22),
                                new ParallelAction(
                                        robot.drive.actionBuilder(start)
                                                .strafeToSplineHeading(new Vector2d(launchPos.position.x, -12), Math.toRadians(-23))
                                                .build()
                                ),
                                robot.intakeStartAction(),
                                //new SleepAction(2),


                                new SequentialAction(
                                        robot.launchAction(),

                                        robot.launchAction(),

                                        robot.launchAction()
                                ),
                                robot.launcher.stopAction(),
                                robot.intake.intakeStopAction(),
                                robot.drive.actionBuilder(launchPos)
                                        .strafeToSplineHeading(new Vector2d(-16, -12), Math.toRadians(0))
                                        .build(),
                                robot.launcher.pointToAction(0),
                                robot.launcher.stopAction()

                        ),
                        robot.launcher.prepShotAction()

                )
        );
    }
}
