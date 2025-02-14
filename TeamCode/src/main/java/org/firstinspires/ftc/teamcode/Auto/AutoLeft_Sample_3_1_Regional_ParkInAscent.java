package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;

@Config
@Autonomous
public class AutoLeft_Sample_3_1_Regional_ParkInAscent extends LinearOpMode {
    public static Vector2d startPos = AutoPositions.Positions.START_LEFT.position;
    public static Vector2d basketPos = new Vector2d(-54.5, -53);
    public static Vector2d basketPosStart = new Vector2d(-54.5, -54);
    public static Vector2d samplePos_1 = AutoPositions.Positions.SAMPLE_1_LEFT.position;
    public static Vector2d samplePos_2 = new Vector2d(-59.5, -38);
    public static Vector2d samplePos_3 = new Vector2d(-55, -38);
    public static Vector2d parkPos = AutoPositions.Positions.ASCENT_ZONE.position;
    @Override
    public void runOpMode() throws InterruptedException {
        //Move to basket () and rotate <-+
        //Lift arm                       |
        //release sample                 |
        //retract arm                    |
        //Move to placed sample ()       |
        //intake sample                  |
        //Repeatx3-----------------------+


        ThunderBot2024 robot = new ThunderBot2024();

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        //Score Preset

                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(120))
                                        .splineToSplineHeading(new Pose2d(basketPosStart, Math.toRadians(45)), Math.toRadians(180))
                                        .build(),
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos)
                        ),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        )
                        //First Cycle

                        ,robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.8),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPosStart, Math.toRadians(45)))
                                        .splineToSplineHeading(new Pose2d(samplePos_1, Math.toRadians(90)), Math.toRadians(90))
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.armUpAction(24),
                                robot.intake.checkForSample("yellow", 7)
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        ),
                        //Second Cycle

                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.8),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPosStart, Math.toRadians(45)))
                                        .splineToSplineHeading(new Pose2d(samplePos_2, Math.toRadians(90)), Math.toRadians(90))
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.armUpAction(24),
                                robot.intake.checkForSample("yellow", 7)
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.5)
                        ),
                        //Third Cycle

                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.7),
                                robot.intake.spinnerAction(1),
                                robot.drive.actionBuilder(new Pose2d(basketPosStart, Math.toRadians(45)))
                                        .splineToSplineHeading(new Pose2d(samplePos_1, Math.toRadians(90)), Math.toRadians(90))
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.wristMoveAction(0.81),
                                robot.intake.armUpAction(16),
                                robot.intake.checkForSample("yellow", 10)
                        ),
                        new SleepAction(1),
                        new ParallelAction(
                                robot.intake.armDownAction(1),
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(135)))
                                        .strafeToSplineHeading(new Vector2d(samplePos_3.x+5, samplePos_3.y-5), Math.toRadians(90))
                                        .build()
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(Intake.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(Intake.Positions.HIGH_BASKET.armPos),
                                robot.drive.actionBuilder(new Pose2d(new Vector2d(samplePos_3.x+5, samplePos_3.y-5), Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build()
                        ),
                        new SleepAction(0.5),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                robot.intake.spinnerAction(-0.5),
                                new SleepAction(0.75)
                        ),
                        robot.intake.spinnerAction(0),
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .splineToSplineHeading(new Pose2d(-48, parkPos.y, Math.toRadians(180)), Math.toRadians(0))
                                        .splineToConstantHeading(parkPos, Math.toRadians(0))
                                        .build(),
                                robot.intake.wristMoveAction(0)
                        ),
                        robot.intake.armUpAction(Intake.ARM_MAX_HORIZONTAL)
                        )
                )
        );
    }
}
