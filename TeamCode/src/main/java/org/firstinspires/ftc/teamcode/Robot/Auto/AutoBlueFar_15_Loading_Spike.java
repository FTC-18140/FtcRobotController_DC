package org.firstinspires.ftc.teamcode.Robot.Auto;

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

@Autonomous(group = AutoBlueFar_WAIT.AUTO_BLUE_FAR_GROUP)
public class AutoBlueFar_15_Loading_Spike extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.FAR_LAUNCH_ZONE_BLUE.position, Math.toRadians(90));
        Pose2d launchPos2 = new Pose2d(AutoPositions.Positions.CENTER_LAUNCH_ZONE_BLUE.position, Math.toRadians(0));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_BASE_BLUE.position, Math.toRadians(90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position, Math.toRadians(110));
        Pose2d intakePos3 = new Pose2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position, Math.toRadians(110));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put("TURRET_ENDING_ANGLE_AUTO", (double) 0);
        blackboard.put("ENDING_ANGLE_INDEXER", (double) 0);

        robot.init(hardwareMap, telemetry, start);
        robot.setColor(ThunderBot2025.Alliance_Color.BLUE);


        // This is the equivalent of init_loop()
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
            robot.launcher.updateVision();
            robot.indexer.updateBallSensors();
            robot.indexer.updateBallStates();
            robot.registerObeliskID();
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }

        waitForStart();

        robot.launcher.setPipeline(1);

        try {
            Actions.runBlocking(
                    new ParallelAction(
                            robot.updateAction(),
                            robot.aimAction(),
                            robot.launcher.prepShotAction(),
                            new SequentialAction(
                                    new RaceAction(
                                            new SequentialAction(
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(start)
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(90))
                                                                    .build()
                                                            // Plan the first shot sequence while driving.
                                                    ),
                                                    // Launch Preloads
                                                    robot.launchAction(),
                                                    robot.intakeStartAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineTo(intakePos.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, 59), Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, 49), Math.toRadians(90)))
                                                                    .setTangent(Math.toRadians(-90))
                                                                    .splineToConstantHeading(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                            //                                        ,
                                                            //                                        // Re-plan the shot sequence with the newly loaded balls
                                                            //                                        robot.planSequenceAction()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts
                                                    robot.launchAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineToConstantHeading(intakePos2.position, Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos2)
                                                                    .setReversed(true)
                                                                    .splineToConstantHeading(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts

                                                    robot.launchAction(),

                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(new Pose2d(launchPos.position, Math.toRadians(90)))
                                                                    .splineTo(intakePos3.position, Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos3)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts

                                                    robot.launchAction(),

                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(new Pose2d(launchPos.position, Math.toRadians(90)))
                                                                    .splineTo(intakePos3.position, Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos3)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts

                                                    robot.launchAction()

                                            ),
                                            new SleepAction(27)
                                    ),
                                    // Park
                                    robot.cancelSequenceAction(),
                                    robot.intake.intakeStopAction(),
                                    robot.drive.actionBuilder(launchPos)
                                            .splineToSplineHeading(new Pose2d(-50, 38, 0), Math.toRadians(90))
                                            .build(),
                                    robot.launcher.pointToAction(0),
                                    new ParallelAction(
                                            robot.holdTurretAction(),
                                            robot.launcher.stopAction()
                                    )
                            )
                    )
            );
        } finally {
            // This block will always run, even if the opmode is stopped prematurely.
            robot.drive.updatePoseEstimate();
            blackboard.put(ThunderBot2025.STARTING_POSE_KEY, robot.drive.localizer.getPose());
            blackboard.put("TURRET_ENDING_ANGLE_AUTO", robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
