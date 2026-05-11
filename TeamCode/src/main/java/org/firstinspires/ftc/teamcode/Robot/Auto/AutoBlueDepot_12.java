package org.firstinspires.ftc.teamcode.Robot.Auto;

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

import org.firstinspires.ftc.teamcode.Robot.Auto.Archive.AutoBlueDepot_Coop;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous(group = AutoBlueDepot_Coop.AUTO_BLUE_DEPOT_GROUP)
public class AutoBlueDepot_12 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_DEPOT.position, Math.toRadians(-45));
        Pose2d launchPos1 = new Pose2d(AutoPositions.Positions.CLOSE_LAUNCH_ZONE_BLUE.position, Math.toRadians(90));
        Pose2d launchPos2 = new Pose2d(new Vector2d(6, 13), Math.toRadians(90));
        Pose2d launchPos3 = new Pose2d(AutoPositions.Positions.PARKING_LAUNCH_ZONE_BLUE.position, Math.toRadians(180));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_GATE_BLUE.position, Math.toRadians(90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.ARTIFACT_CENTER_BLUE.position, Math.toRadians(90));
        Pose2d intakePos3 = new Pose2d(AutoPositions.Positions.ARTIFACT_BASE_BLUE.position, Math.toRadians(90));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, (double) 0);
        blackboard.put(AutoRedDepot_12.ENDING_ANGLE_INDEXER_KEY, (double) 0);

        robot.init(hardwareMap, telemetry, start);
        robot.setColor(ThunderBot2025.Alliance_Color.BLUE);


        // This is the equivalent of init_loop()
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
//            robot.indexer.updateBallSensors();
//            robot.indexer.updateBallStates();
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
                                                                    .strafeToSplineHeading(launchPos1.position, Math.toRadians(90))
                                                                    .build()
                                                    ),
                                                    // Launch Preloads
                                                    robot.launchAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos1)
                                                                    .setTangent(Math.toRadians(-90))
                                                                    .splineToConstantHeading(intakePos.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, 51.5), Math.toRadians(90), new TranslationalVelConstraint(20))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, 51.5), Math.toRadians(90)))
                                                                    .setReversed(true)
                                                                    .splineToConstantHeading(launchPos2.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),
                                                    robot.intakeStopAction(),
                                                    // Launch Preloads
                                                    robot.launchAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos2)
                                                                    .splineToConstantHeading(intakePos2.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos2.position.x, 58), Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos2.position.x, 58), Math.toRadians(90)))
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos2.position, Math.toRadians(0))
                                                                    .build()
//                                                            ,
//                                                            // Re-plan the shot sequence with the newly loaded balls
//                                                            robot.planSequenceAction()
                                                    ),
                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts
                                                    robot.launchAction(),

                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(new Pose2d(launchPos2.position, Math.toRadians(180)))
                                                                    .splineTo(intakePos3.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos3.position.x, 58), Math.toRadians(90))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos3.position.x, 58), Math.toRadians(90)))
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos3.position, Math.toRadians(0))
                                                                    .build()
//                                                            ,
//                                                            // Re-plan the shot sequence with the newly loaded balls
//                                                            robot.planSequenceAction()
                                                    ),
                                                    robot.intakeStopAction(),
                                                    // Launch 3rd set of Artifacts
                                                    robot.launchAction()
                                            ),
                                            new SleepAction(27)
                                    ),
                                    robot.cancelSequenceAction(),
                                    robot.intakeStopAction(),
                                    robot.launcher.pointToAction(0),
                                    robot.drive.actionBuilder(launchPos3)
                                            .setReversed(true)
                                            .splineToSplineHeading(new Pose2d(launchPos3.position.x + 2, launchPos3.position.y, launchPos3.heading.toDouble()), Math.toRadians(0))
                                            .build(),
                                    new ParallelAction(
                                            robot.holdTurretAction(),
                                            robot.launcher.stopAction()
                                    )
                            )
                    )
            );
        } finally {
            robot.drive.updatePoseEstimate();
            blackboard.put(ThunderBot2025.STARTING_POSE_KEY, robot.drive.localizer.getPose());
            blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
