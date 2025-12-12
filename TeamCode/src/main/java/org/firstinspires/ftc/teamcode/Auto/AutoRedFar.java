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
public class AutoRedFar extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.FAR_LAUNCH_ZONE_RED.position, Math.toRadians(-23));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_BASE_RED.position, Math.toRadians(-90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.ARTIFACT_CENTER_RED.position, Math.toRadians(-90));

        ThunderBot2025 robot = new ThunderBot2025();

        robot.init(hardwareMap, telemetry, start);


        // This is the equivalent of init_loop()
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
            robot.launcher.updateVision();
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }

        waitForStart();

        robot.setColor(ThunderBot2025.Alliance_Color.BLUE);

        try {
            Actions.runBlocking(
                    new ParallelAction(
                            robot.updateAction(),
                            new SequentialAction(
                                    new RaceAction(
                                            new SequentialAction(
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(start)
                                                                    .strafeToSplineHeading(new Vector2d(launchPos.position.x, -12), Math.toRadians(-23))
                                                                    .build(),
                                                            // Plan the first shot sequence while driving.
                                                            robot.planSequenceAction()
                                                    ),
                                                    robot.intake.intakeStartAction(),
                                                    // Launch Preloads
                                                    new SequentialAction(
                                                            robot.launchAction(),
                                                            robot.launchAction(),
                                                            robot.launchAction()
                                                    ),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineToSplineHeading(intakePos, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, -52), Math.toRadians(-90), new TranslationalVelConstraint(4))
                                                                    .build(),
                                                            new SequentialAction(
                                                                    robot.seekToSlotAction(0), // Move to the first intake slot
                                                                    robot.waitForBallAndCycleAction(), // Wait for a ball, then cycle
                                                                    robot.waitForBallAndCycleAction(), // Wait for the next ball, then cycle
                                                                    new SleepAction(0.75)
                                                                    // The third ball will be loaded but we won't cycle away from it
                                                            )
                                                    ),
                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, -52), Math.toRadians(-90)))
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-23))
                                                                    .build(),
                                                            // Re-plan the shot sequence with the newly loaded balls
                                                            robot.planSequenceAction()
                                                    ),
                                                    // Launch 2nd set of Artifacts
                                                    new SequentialAction(
                                                            robot.launchAction(),
                                                            robot.launchAction(),
                                                            robot.launchAction()
                                                    ),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineToSplineHeading(intakePos2, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos2.position.x, -52), Math.toRadians(-90), new TranslationalVelConstraint(4))
                                                                    .build(),
                                                            new SequentialAction(
                                                                    robot.seekToSlotAction(0), // Move to the first intake slot
                                                                    robot.waitForBallAndCycleAction(), // Wait for a ball, then cycle
                                                                    robot.waitForBallAndCycleAction(), // Wait for the next ball, then cycle
                                                                    new SleepAction(0.75)
                                                                    // The third ball will be loaded but we won't cycle away from it
                                                            )
                                                    ),
                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos2.position.x, -52), Math.toRadians(-90)))
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-23))
                                                                    .build(),
                                                            // Re-plan the shot sequence with the newly loaded balls
                                                            robot.planSequenceAction()
                                                    ),
                                                    // Launch 2nd set of Artifacts
                                                    new SequentialAction(
                                                            robot.launchAction(),
                                                            robot.launchAction(),
                                                            robot.launchAction()
                                                    )

                                            ),
                                            new SleepAction(27)
                                    ),
                                    // Park
                                    robot.intake.intakeStopAction(),
                                    robot.drive.actionBuilder(launchPos)
                                            .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(0))
                                            .build(),
                                    robot.launcher.pointToAction(0),
                                    robot.launcher.stopAction()
                            ),
                            robot.launcher.prepShotAction(),
                            robot.aimAction()
                    )
            );
        } finally {
            // This block will always run, even if the opmode is stopped prematurely.
            blackboard.put("ENDING_POSITION_AUTO", robot.drive.localizer.getPose());
            blackboard.put("TURRET_ENDING_ANGLE_AUTO", robot.launcher.getTurretAngle());
        }
    }
}
