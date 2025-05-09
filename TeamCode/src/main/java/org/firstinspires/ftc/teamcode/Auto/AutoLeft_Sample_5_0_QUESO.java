package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

@Autonomous
public class AutoLeft_Sample_5_0_QUESO extends LinearOpMode {
    public static Vector2d startPos = new Vector2d(-39, -60);
    public static Vector2d basketPos = new Vector2d(-55, -52);

    public static Vector2d samplePos_1 = new Vector2d(-48, -40.25);
    public static Vector2d samplePos_2 = new Vector2d(-58, -41.25);
    public static Vector2d samplePos_3 = new Vector2d(-55.5, -29.25);
    public static Vector2d parkPos = AutoPositions.Positions.ASCENT_ZONE.position;



    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));
        TrajectoryActionBuilder scan = robot.drive.actionBuilder(new Pose2d(new Vector2d(-36, -13),Math.toRadians(0)))
                .strafeTo(new Vector2d(-36, -8), new TranslationalVelConstraint(2.0));

        telemetry.update();

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(120))
                                        .splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(210))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                new SleepAction(0.25)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        //First Cycle
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .strafeToSplineHeading(samplePos_1, Math.toRadians(90), new TranslationalVelConstraint(40.0))
                                        .build(),
                                robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                                robot.intake.wristMoveAction(IntakeClaw.Positions.READY_TO_INTAKE.wristPos),
                                robot.intake.armDownAction(17)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.3),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.25),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_1, Math.toRadians(90)))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new SleepAction(0.5),
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
                                robot.intake.wristMoveAction(IntakeClaw.Positions.READY_TO_INTAKE.wristPos),
                                robot.intake.armDownAction(17)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.3),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.25),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_2, Math.toRadians(90)))
                                        .setTangent(Math.toRadians(120))
                                        .strafeToSplineHeading(basketPos, Math.toRadians(45))
                                        .build(),
                                robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                new SleepAction(0.25),
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
                                robot.intake.wristMoveAction(IntakeClaw.Positions.READY_TO_INTAKE.wristPos),
                                robot.intake.pivotAction(0.83)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.3),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(samplePos_3, Math.toRadians(150)))
                                        .setTangent(Math.toRadians(0))
                                        .splineToSplineHeading(new Pose2d(samplePos_3.x+6, samplePos_3.y-6, Math.toRadians(90)), Math.toRadians(-90))
                                        .splineToSplineHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(-135))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                        robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                                )

                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                new SleepAction(0.25),
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        //Submersible
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                                        .setTangent(Math.toRadians(90))
                                        .splineToSplineHeading(new Pose2d(-36, -16, Math.toRadians(0)), Math.toRadians(0))
                                        .splineToConstantHeading(new Vector2d(-12, -13), Math.toRadians(0))
                                        .build(),
                                new SequentialAction(
                                        robot.intake.armDownAction(1),
                                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                                        new SleepAction(1.5),
                                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                        robot.intake.elbowAction(14),
                                        robot.intake.armUpAction(IntakeClaw.ARM_MAX_HORIZONTAL)
                                )
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new RaceAction(
                                robot.updatePixyAction(),
                                robot.alignToColorAction(4)
                        ),
                        robot.intake.elbowAction(IntakeClaw.Positions.READY_TO_INTAKE.elbowPos),
                        new SleepAction(0.5),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.5),
                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        new SleepAction(0.1),
                        new InstantAction(() -> robot.drive.updatePoseEstimate()),
                        new ParallelAction(
                                robot.drive.actionBuilder(new Pose2d(-12, -10, 0))
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(-135))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(0.75),
                                        robot.intake.presetAction(IntakeClaw.Positions.HIGH_BASKET),
                                        robot.intake.armUpAction(IntakeClaw.Positions.HIGH_BASKET.armPos)
                                )
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                                new SleepAction(0.25)
                        ),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new SleepAction(0.5),

                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE),
                        robot.intake.armDownAction(1),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MIN)
                )
        ));
    }
}
