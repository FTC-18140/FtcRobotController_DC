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

import org.firstinspires.ftc.teamcode.Robot.Auto.Archive.AutoRedDepot_Coop;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous(group = AutoRedDepot_Coop.AUTO_RED_DEPOT_GROUP)
public class AutoRedDepot_9_SORTING extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_DEPOT.position, Math.toRadians(45));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.CENTER_LAUNCH_ZONE_RED.position, Math.toRadians(-90));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_GATE_RED.position, Math.toRadians(-90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.ARTIFACT_CENTER_RED.position, Math.toRadians(-90));

        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, (double) 0);
        blackboard.put(AutoRedDepot_12.ENDING_ANGLE_INDEXER_KEY, (double) 0);

        robot.init(hardwareMap, telemetry, start);
        robot.launcher.setTurretStart(-45);
        robot.setColor(ThunderBot2025.Alliance_Color.RED);


        // This is the equivalent of init_loop()
        while (opModeInInit()) {
            // Code here runs repeatedly during init phase.  Need to be looking at ObeliskID
            robot.launcher.updateVision();
//            robot.indexer.updateBallSensors();
//            robot.indexer.updateBallStates();
            robot.registerObeliskID();
            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }

        waitForStart();

        robot.launcher.setPipeline(2);

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
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),
                                                    // Launch Preloads
                                                    robot.sortAndLaunchAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .setTangent(Math.toRadians(90))
                                                                    .splineToSplineHeading(intakePos, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, -49), Math.toRadians(-90), new TranslationalVelConstraint(12))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, -49), Math.toRadians(-90)))
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),
                                                    robot.intakeStopAction(),
                                                    // Launch Preloads
                                                    robot.sortAndLaunchAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .setTangent(Math.toRadians(90))
                                                                    .splineToSplineHeading(intakePos2, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos2.position.x, -49), Math.toRadians(-90), new TranslationalVelConstraint(12))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos2.position.x, -49), Math.toRadians(-90)))
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(-90))
                                                                    .build()
//                                                            ,
//                                                            // Re-plan the shot sequence with the newly loaded balls
//                                                            robot.planSequenceAction()
                                                    ),
                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts
                                                    robot.sortAndLaunchAction()
                                            ),
                                            new SleepAction(27)
                                    ),
                                    robot.cancelSequenceAction(),
                                    robot.intakeStopAction(),
                                    robot.drive.actionBuilder(launchPos)
                                            .strafeToSplineHeading(new Vector2d(38, -12), Math.toRadians(0))
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
            robot.drive.updatePoseEstimate();
            blackboard.put(ThunderBot2025.STARTING_POSE_KEY, robot.drive.localizer.getPose());
            blackboard.put(AutoRedDepot_12.TURRET_ENDING_ANGLE_AUTO_KEY, robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
