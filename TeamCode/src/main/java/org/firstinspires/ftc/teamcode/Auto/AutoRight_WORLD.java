package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

@Autonomous
public class AutoRight_WORLD extends LinearOpMode {
    public static Vector2d startPos = AutoPositions.Positions.START_RIGHT.position;
    public static Vector2d basketPos = new Vector2d(56, -56);

    public static Vector2d samplePos_1 = new Vector2d(48, -45);
    public static Vector2d samplePos_2 = new Vector2d(59, -43);
    public static Vector2d samplePos_3 = new Vector2d(54.5, -32.5);
    public static Vector2d deposit = new Vector2d(56, -52);
    public static Vector2d pickup = new Vector2d(47, -50.5);
    public static Vector2d parkPos = AutoPositions.Positions.ASCENT_ZONE.position;

    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        TrajectoryActionBuilder toChamber = robot.drive.actionBuilder(new Pose2d(pickup, Math.toRadians(-90)))
                .setTangent(Math.toRadians(170))
                .splineToSplineHeading(new Pose2d(7, -42, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(7, -31), Math.toRadians(90));
        telemetry.update();

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(
                new ParallelAction(
                    robot.intake.updateAction(),
                    new SequentialAction(
                        new ParallelAction(
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING.armPos),
                                new SequentialAction(
                                        new SleepAction(0.25),
                                        robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                                .setTangent(Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(9, -42), Math.toRadians(90))
                                                .splineToConstantHeading(new Vector2d(9, -31), Math.toRadians(90))
                                                .build()
                                )
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        //First Cycle
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(9,-31, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(10, -48), Math.toRadians(0))
                                        .splineToConstantHeading(samplePos_1, Math.toRadians(0))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.75),
                                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                        robot.intake.armUpAction(17)
                                )
                        ),
                            robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.25),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(100))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.LOW_BASKET),
                                robot.intake.armDownAction(1)
                        ),
                        new SleepAction(0.25),
                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                new SleepAction(0.5)
                        ),
                        //Second Cycle
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(100)))
                                        .strafeToSplineHeading(samplePos_2, Math.toRadians(90))
                                        .build(),
                                robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                                robot.intake.armUpAction(17)
                        ),
                            robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.99),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.35),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(90)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(100))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.LOW_BASKET),
                                robot.intake.armDownAction(1)
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                new SleepAction(0.5),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),

                        //Third Cycle

                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(100)))
                                        .strafeToSplineHeading(samplePos_3, Math.toRadians(30))
                                        .build(),
                                robot.intake.armUpAction(10),
                                robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                                robot.intake.pivotAction(0.17)
                        ),
                            robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.25),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.35),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(45)))
                                        .strafeToSplineHeading(deposit, Math.toRadians(100))
                                        .build(),
                                robot.intake.armDownAction(1),
                                robot.intake.presetAction(IntakeClaw.Positions.LOW_BASKET)
                        ),
                        new SleepAction(0.2),
                        new ParallelAction(
                                new SleepAction(0.5),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        new ParallelAction(
                                robot.intake.presetAction(IntakeClaw.Positions.INTAKE_SPECIMEN),
                                robot.intake.armUpAction(IntakeClaw.Positions.INTAKE_SPECIMEN.armPos),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                robot.drive.actionBuilder(new Pose2d(deposit, Math.toRadians(100)))
                                        .splineToSplineHeading(new Pose2d(52, samplePos_1.y, Math.toRadians(-90)), Math.toRadians(180))
                                        .splineToConstantHeading(pickup, Math.toRadians(-90))
                                        .build()
                        ),
                        new SleepAction(0.25),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.25),
                        robot.intake.wristMoveAction(0.25),
                        new ParallelAction(
                                toChamber.build(),
                                new SequentialAction(
                                        new SleepAction(1.0),
                                        robot.intake.presetAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING),
                                        robot.intake.armUpAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING.armPos)
                                )
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(7,-31, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(14, -50), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(46, -52), Math.toRadians(0))
                                        .build(),
                                new SequentialAction(
                                        robot.intake.armDownAction(1),
                                        new SleepAction(0.5),
                                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE)

                                )

                        ),
                            robot.intake.wristMoveAction(IntakeClaw.WRIST_MIN)

                    )
                )
        );
    }
}
