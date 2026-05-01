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

import org.firstinspires.ftc.teamcode.Robot.Auto.Archive.AutoBlueFar_WAIT;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous(group = AutoRedFar_9.AUTO_RED_FAR_GROUP)
public class AutoRedFar_15_3S_1L extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.FAR_LAUNCH_ZONE_RED.position, Math.toRadians(-45));
        Pose2d launchPos2 = new Pose2d(AutoPositions.Positions.CENTER_LAUNCH_ZONE_RED.position, Math.toRadians(0));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_BASE_RED.position, Math.toRadians(-90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.LOADING_ZONE_RED.position, Math.toRadians(-110));
        Pose2d intakePos3 = new Pose2d(AutoPositions.Positions.ARTIFACT_CENTER_RED.position, Math.toRadians(-90));
        Pose2d intakePos4 = new Pose2d(AutoPositions.Positions.ARTIFACT_GATE_RED.position, Math.toRadians(-90));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, (double) 0);
        blackboard.put("ENDING_ANGLE_INDEXER", (double) 0);

        robot.init(hardwareMap, telemetry, start);
        robot.setColor(ThunderBot2025.Alliance_Color.RED);


        // This is the equivalent of init_loop()
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
            robot.indexer.updateBallSensors();
            robot.indexer.updateBallStates();
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
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-45))
                                                                    .build()
                                                            // Plan the first shot sequence while driving.
                                                    ),
                                                    // Launch Preloads
                                                    robot.launchAction(),
                                                    robot.intakeStartAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .setTangent(-45)
                                                                    .splineToSplineHeading(intakePos, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, -59), Math.toRadians(-90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, -59), Math.toRadians(-90)))
                                                                    .setTangent(Math.toRadians(90))
                                                                    .splineToSplineHeading(launchPos, Math.toRadians(0))
                                                                    .build(),
                                                            // Launch 2nd set of Artifacts
                                                            robot.launchAction()
                                                    ),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineToSplineHeading(intakePos2, Math.toRadians(-120))
                                                                    .turnTo(Math.toRadians(-90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos2)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(135))
                                                                    .build(),
                                                            // Launch 2nd set of Artifacts

                                                            robot.launchAction()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    robot.intakeStartAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .setTangent(0)
                                                                    .splineToConstantHeading(intakePos3.position, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos3.position.x, -59), Math.toRadians(-90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos3.position.x, -59), Math.toRadians(-90)))
                                                                    .setReversed(true)
                                                                    .splineToConstantHeading(intakePos3.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(launchPos2.position, Math.toRadians(0))
                                                                    .build(),

                                                            // Launch 2nd set of Artifacts

                                                            robot.launchAction()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    robot.intakeStartAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(new Pose2d(launchPos2.position, Math.toRadians(-90)))
                                                                    .splineToConstantHeading(intakePos4.position, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos4.position.x, -53), Math.toRadians(-90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos4.position.x, -53), Math.toRadians(-90)))
                                                                    .setReversed(true)
                                                                    .splineToConstantHeading(launchPos2.position, Math.toRadians(90))
                                                                    .build()
                                                            //                                            ,
                                                            //                                            // Re-plan the shot sequence with the newly loaded balls
                                                            //                                            robot.planSequenceAction()
                                                    ),
                                                    robot.intakeStopAction(),

                                                    // Launch 2nd set of Artifacts

                                                    robot.launchAction()

                                            ),
                                            new SleepAction(27.5)
                                    ),
                                    // Park
                                    robot.cancelSequenceAction(),
                                    robot.intakeStopAction(),

                                    robot.launcher.pointToAction(0),
                                    new ParallelAction(
                                            robot.drive.actionBuilder(new Pose2d(launchPos2.position, Math.toRadians(90)))
                                                    .setTangent(Math.toRadians(180))
                                                    .splineToSplineHeading(new Pose2d(-12, -12, 0), Math.toRadians(180))
                                                    .build(),
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
            blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
