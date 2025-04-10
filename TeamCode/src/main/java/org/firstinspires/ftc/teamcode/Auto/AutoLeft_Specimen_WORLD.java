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
public class AutoLeft_Specimen_WORLD extends LinearOpMode {
    public static Vector2d startPos = AutoPositions.Positions.START_LEFT.position;
    public static Vector2d basketPos = new Vector2d(-56, -56);

    public static Vector2d samplePos_1 = new Vector2d(-49, -40);
    public static Vector2d samplePos_2 = new Vector2d(-59, -40);
    public static Vector2d samplePos_3 = new Vector2d(-55.5, -29);
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
                        new ParallelAction(
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_CHAMBER_SCORING.armPos),
                                new SequentialAction(
                                        new SleepAction(0.25),
                                        robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                            .setTangent(Math.toRadians(90))
                                            .splineToConstantHeading(new Vector2d(-7, -42), Math.toRadians(90))
                                            .splineToConstantHeading(new Vector2d(-7, -31), Math.toRadians(90))
                                            .build()
                                )
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        //First Cycle
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-7,-42, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-10, -48), Math.toRadians(180))
                                        .splineToConstantHeading(samplePos_1, Math.toRadians(180))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                                        robot.intake.armUpAction(17)
                                )
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                new SleepAction(0.25)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        //Second Cycle
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .strafeToSplineHeading(samplePos_2, Math.toRadians(90))
                                        .build(),
                                robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                                robot.intake.armDownAction(17),
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX)
                        ),
                        new SleepAction(0.8),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(120))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new ParallelAction(
                                new SleepAction(0.5),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        //Third Cycle
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .strafeToSplineHeading(samplePos_3, Math.toRadians(150))
                                        .build(),
                                robot.intake.armDownAction(10),
                                robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                                robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                                robot.intake.pivotAction(0.83)
                        ),
                        new SleepAction(0.1),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(samplePos_3.x+6, samplePos_3.y-6, Math.toRadians(90)), Math.toRadians(-90))
                                        .splineToSplineHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(-135))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new ParallelAction(
                                new SleepAction(0.25),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MIN)
                )
        ));
    }
}
