package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

@Autonomous
public class AutoRight_Claw extends LinearOpMode {
    public static Vector2d startPos = AutoPositions.Positions.START_RIGHT.position;
    public static Vector2d basketPos = new Vector2d(56, -56);

    public static Vector2d samplePos_1 = new Vector2d(48, -42);
    public static Vector2d samplePos_2 = new Vector2d(48, -40);
    public static Vector2d samplePos_3 = new Vector2d(52, -38);
    public static Vector2d deposit = new Vector2d(48, -46);
    public static Vector2d parkPos = AutoPositions.Positions.ASCENT_ZONE.position;

    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));
        telemetry.update();

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        robot.intake.presetAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING),
                        robot.intake.armUpAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING.armPos),
                        new SleepAction(0.5),
                        robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                .strafeTo(new Vector2d(7, -56))
                                .strafeTo(new Vector2d(7, -31))
                                .build(),
                        new ParallelAction(
                            robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                            robot.drive.actionBuilder(new Pose2d(10,-42, Math.toRadians(90)))
                                    .strafeTo(new Vector2d(10, -54))
                                    .build(),
                            robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                            robot.intake.armDownAction(1)
                        ),
                        //First Cycle

                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                robot.drive.actionBuilder(new Pose2d(new Vector2d(10, -54), Math.toRadians(90)))
                                        .strafeToSplineHeading(samplePos_1, Math.toRadians(90))
                                        .build(),
                                robot.intake.armUpAction(17),
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX)
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.7),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(-60))
                                        .build(),
                                robot.intake.armDownAction(1)
                        ),
                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                new SleepAction(0.1)
                        ),
                        //Second Cycle
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(-60)))
                                        .strafeToSplineHeading(samplePos_2, Math.toRadians(65))
                                        .build(),
                                robot.intake.armUpAction(19),
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                                robot.intake.pivotAction(0.42)
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.7),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(65)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(-60))
                                        .build(),
                                robot.intake.armDownAction(1)
                        ),
                        new ParallelAction(
                                new SleepAction(0.1),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        //Third Cycle

                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(-60)))
                                        .strafeToSplineHeading(samplePos_3, Math.toRadians(45))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        robot.intake.armUpAction(22)
                                ),
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                                robot.intake.pivotAction(0.25)
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.7),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(45)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(-60))
                                        .build(),
                                robot.intake.armDownAction(1)
                        ),
                        new ParallelAction(
                                new SleepAction(0.25),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        new ParallelAction(
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                                robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                                robot.intake.armDownAction(1),
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(-90)))
                                        .strafeTo(new Vector2d(samplePos_3.x, samplePos_3.y))
                                        .build()
                        )

                )
        ));
    }
}
