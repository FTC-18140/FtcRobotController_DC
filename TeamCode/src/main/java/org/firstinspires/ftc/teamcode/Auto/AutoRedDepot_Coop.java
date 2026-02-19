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

@Autonomous(group = "AutoRedDepot")
public class AutoRedDepot_Coop extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(AutoPositions.Positions.START_RED_DEPOT.position, Math.toRadians(45));
        Pose2d launchPos1 = new Pose2d(AutoPositions.Positions.CLOSE_LAUNCH_ZONE_RED.position, Math.toRadians(-90));
        Pose2d launchPos2 = new Pose2d(AutoPositions.Positions.CENTER_LAUNCH_ZONE_RED.position, Math.toRadians(-90));
        Pose2d launchPos3 = new Pose2d(AutoPositions.Positions.PARKING_LAUNCH_ZONE_RED.position, Math.toRadians(-90));
        Pose2d intakePos = new Pose2d(AutoPositions.Positions.ARTIFACT_GATE_RED.position, Math.toRadians(-90));
        Pose2d intakePos2 = new Pose2d(AutoPositions.Positions.ARTIFACT_CENTER_RED.position, Math.toRadians(-90));
        Pose2d gatePos = new Pose2d(AutoPositions.Positions.GATE_RED.position, Math.toRadians(-90));


        ThunderBot2025 robot = new ThunderBot2025();
        blackboard.put("TURRET_ENDING_ANGLE_AUTO", (double) 0);
        blackboard.put("ENDING_ANGLE_INDEXER", (double) 0);

        robot.init(hardwareMap, telemetry, start);
        robot.launcher.setTurretStart(-45);
        robot.setColor(ThunderBot2025.Alliance_Color.RED);


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
        robot.runtime.reset();

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
                                                                    .strafeToSplineHeading(launchPos1.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),
                                                    robot.waitForTime(4),
                                                    // Launch Preloads
                                                    robot.startSequenceAction(),
                                                    robot.waitForSequenceEndAction(),
                                                    robot.intakeStartAction(),
                                                    // Grab next 3 artifacts using intelligent, sensor-based actions
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(launchPos1)
                                                                    .setTangent(90)
                                                                    .splineToConstantHeading(intakePos.position, Math.toRadians(-90))
                                                                    .splineToConstantHeading(new Vector2d(intakePos.position.x, -51), Math.toRadians(-90), new TranslationalVelConstraint(20))
                                                                    .waitSeconds(1)
                                                                    .build(),
                                                            robot.indexerFullAction()
                                                    ),
                                                    robot.waitForTime(13.5),
                                                    robot.intakeStopAction(),
                                                    new RaceAction(
                                                            robot.drive.actionBuilder(new Pose2d(new Vector2d(intakePos.position.x, -51), Math.toRadians(-90)))
                                                                    .setTangent(Math.toRadians(90))
                                                                    .splineToConstantHeading(gatePos.position, Math.toRadians(-90))
                                                                    .build()
                                                    ),
                                                    new SleepAction(2),
                                                    new ParallelAction(
                                                            robot.drive.actionBuilder(new Pose2d(gatePos.position, Math.toRadians(-90)))
                                                                    .setReversed(true)
                                                                    .splineTo(launchPos3.position, Math.toRadians(90))
                                                                    .build(),
                                                            robot.planSequenceAction()
                                                    ),
                                                    robot.waitForTime(21.5),
                                                    // Launch Preloads
                                                    robot.startSequenceAction(),
                                                    robot.waitForSequenceEndAction()
                                            ),
                                            new SleepAction(27)
                                    ),
                                    robot.cancelSequenceAction(),
                                    robot.intakeStopAction(),
                                    robot.launcher.pointToAction(0),
                                    new ParallelAction(
                                            robot.drive.actionBuilder(launchPos3)
                                                            .setReversed(true)
                                                                    .splineTo(new Vector2d(52, -12), 0)
                                                                            .build(),
                                            robot.holdTurretAction(),
                                            robot.launcher.stopAction()
                                    )
                            )
                    )
            );
        }finally{
            robot.drive.updatePoseEstimate();
            blackboard.put("ENDING_POSITION_AUTO", robot.drive.localizer.getPose());
            blackboard.put("TURRET_ENDING_ANGLE_AUTO", robot.launcher.getTurretAngle());
//            ThunderBot2025.starting_position = robot.drive.localizer.getPose();
//            ThunderBot2025.starting_turret_angle = robot.launcher.getTurretAngle();
        }
    }
}
