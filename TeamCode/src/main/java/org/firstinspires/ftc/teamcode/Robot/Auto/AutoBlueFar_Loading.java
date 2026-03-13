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

import org.firstinspires.ftc.teamcode.Robot.ThunderBot2025;

@Autonomous(group = AutoBlueFar_WAIT.AUTO_BLUE_FAR_GROUP)
public class AutoBlueFar_Loading extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_BLUE_FAR.position, Math.toRadians(0));
        Pose2d launchPos = new Pose2d(AutoPositions.Positions.FAR_LAUNCH_ZONE_BLUE.position, Math.toRadians(90));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position, Math.toRadians(110));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position, Math.toRadians(110));
        Pose2d preintakePos1 = new Pose2d(new Vector2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position.x, 40), Math.toRadians(90));
        Pose2d preintakePos2 = new Pose2d(new Vector2d(AutoPositions.Positions.LOADING_ZONE_BLUE.position.x+5, 51), Math.toRadians(90));

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
                                    robot.indexer.homeAction(),
                                    new RaceAction(
                                            new SequentialAction(
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(start)
                                                                    .strafeToSplineHeading(launchPos.position, Math.toRadians(90))
                                                                    .build()
                                                            // Plan the first shot sequence while driving.
                                                    ),
                                                    // Launch Preloads
                                                    robot.spamAction(),
                                                    robot.intakeStartAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineTo(preintakePos1.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(preintakePos2.position, Math.toRadians(90))
                                                                    .splineTo(intakePos.position, Math.toRadians(110), new TranslationalVelConstraint(20))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts
                                                    robot.spamAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineTo(preintakePos1.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(preintakePos2.position, Math.toRadians(90))
                                                                    .splineTo(intakePos.position, Math.toRadians(110), new TranslationalVelConstraint(20))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts

                                                    robot.planSequenceAction(),
                                                    robot.startSequenceAction(),
                                                    robot.waitForSequenceEndAction(),

                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos)
                                                                    .splineTo(preintakePos1.position, Math.toRadians(90))
                                                                    .splineToConstantHeading(preintakePos2.position, Math.toRadians(90))
                                                                    .splineTo(intakePos.position, Math.toRadians(110), new TranslationalVelConstraint(20))
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),

                                                    // Drive to launch spot
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(intakePos)
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),

                                                    robot.intakeStopAction(),
                                                    // Launch 2nd set of Artifacts

                                                    robot.planSequenceAction(),
                                                    robot.startSequenceAction(),
                                                    robot.waitForSequenceEndAction()

                                            ),
                                            new SleepAction(26)
                                    ),
                                    // Park
                                    robot.cancelSequenceAction(),
                                    robot.intakeStopAction(),

                                    robot.launcher.pointToAction(0),
                                    new ParallelAction(
                                            robot.drive.actionBuilder(launchPos)
                                                    .setTangent(Math.toRadians(0))
                                                    .splineToConstantHeading(new Vector2d(-36, 24), Math.toRadians(0))
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
            blackboard.put("TURRET_ENDING_ANGLE_AUTO", robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
