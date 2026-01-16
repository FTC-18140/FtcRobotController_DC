package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put("TURRET_ENDING_ANGLE_AUTO", (double) 0);
        blackboard.put("ENDING_ANGLE_INDEXER", (double) 0);

        robot.init(hardwareMap, telemetry, start);
        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.RED);

        Actions.runBlocking(
                new ParallelAction(
                        robot.updateAction(),
//                        robot.aimAction(),
                        new SequentialAction(
                                robot.intakeStartAction(), // Move to the first intake slot
                            new RaceAction(
                                new SequentialAction(
                                        robot.waitForBallAndCycleAction(), // Wait for a ball, then cycle
                                        robot.indexerIsAtTargetAction(),
                                        robot.waitForBallAndCycleAction(), // Wait for a ball, then cycle
                                        robot.indexerIsAtTargetAction(),
                                        robot.waitForBallAction()
                                )
//                                robot.indexerFullAction()
                            ),
                                robot.intake.intakeStopAction()
                        )
                )
        );

        //robot.setStartPosForTeleop(robot.drive.localizer.getPose(), 0);

    }
}
