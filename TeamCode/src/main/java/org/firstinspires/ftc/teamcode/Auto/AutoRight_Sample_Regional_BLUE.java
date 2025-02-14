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
public class AutoRight_Sample_Regional_BLUE extends LinearOpMode {
    public static Vector2d startPos = new Vector2d(15,-60);
    public static Vector2d basketPos = new Vector2d(-40,-54);
    public static Vector2d samplePos = new Vector2d(50,-40);
    public static Vector2d parkPos = new Vector2d(44,-58);
    @Override
    public void runOpMode() throws InterruptedException {
        //Move to basket () and rotate <-+
        //Lift arm                       |
        //release sample                 |
        //retract arm                    |
        //Move to placed sample ()       |
        //intake sample                  |
        //Repeat-------------------------+
        //park ()

        ThunderBot2024 robot = new ThunderBot2024();

        robot.init(hardwareMap,telemetry, 0);
        robot.drive.pose = new Pose2d(startPos,Math.toRadians(90));

        waitForStart();
        robot.intake.start();

        Actions.runBlocking(new ParallelAction(
                robot.intake.updateAction(),
                new SequentialAction(
                        robot.intake.presetAction(Intake.Positions.HIGH_CHAMBER_SCORING_AUTO),
                        robot.intake.armUpAction(Intake.Positions.HIGH_CHAMBER_SCORING_AUTO.armPos),
                        new SleepAction(0.5),
                        robot.drive.actionBuilder(new Pose2d(startPos, Math.toRadians(90)))
                                .strafeTo(new Vector2d(7, -56))
                                .strafeTo(new Vector2d(7, -34))
                                .build(),
                        robot.intake.clawAction(0),
                        robot.drive.actionBuilder(new Pose2d(10,-42, Math.toRadians(90)))
                                .strafeTo(new Vector2d(7, -54))
                                .build(),
                        robot.intake.presetAction(Intake.Positions.READY_TO_INTAKE),

                        robot.drive.actionBuilder(new Pose2d(new Vector2d(10, -58), Math.toRadians(90)))
                                .strafeToSplineHeading(new Vector2d(32, -40), Math.toRadians(45))
                                .build(),
                        robot.intake.wristMoveAction(0.79),
                        new ParallelAction(
                                robot.intake.spinnerAction(1),
                                robot.intake.armUpAction(25),
                                robot.intake.checkForSample("blue", 2)
                        ),
                        new SleepAction(0.7),
                        new ParallelAction(
                            robot.intake.spinnerAction(0),
                            robot.intake.armDownAction(10),
                            robot.intake.wristMoveAction(0.5),
                            robot.drive.actionBuilder(new Pose2d(new Vector2d(30, -42), Math.toRadians(45)))
                                    .strafeToSplineHeading(new Vector2d(38, -48), Math.toRadians(-30))
                                    .build()
                        ),
                        robot.intake.spinnerAction(-1),
                        new SleepAction(1)
                        //sample 2
                        ,
                        robot.drive.actionBuilder(new Pose2d(new Vector2d(38, -48), Math.toRadians(-30)))
                                .strafeToSplineHeading(new Vector2d(38, -42), Math.toRadians(45))
                                .build(),
                        robot.intake.wristMoveAction(0.79),
                        new ParallelAction(
                                robot.intake.spinnerAction(1),
                                robot.intake.armUpAction(28),
                                robot.intake.checkForSample("blue", 2)
                        ),
                        new SleepAction(0.7),
                        new ParallelAction(
                                robot.intake.spinnerAction(0),
                                robot.intake.armDownAction(10),
                                robot.intake.wristMoveAction(0.5),
                                robot.drive.actionBuilder(new Pose2d(new Vector2d(44, -42), Math.toRadians(45)))
                                        .strafeToSplineHeading(new Vector2d(38, -48), Math.toRadians(-30))
                                        .build()
                        ),
                        robot.intake.spinnerAction(-1),
                        new SleepAction(1)

                        //sample 3
                        ,
                        robot.drive.actionBuilder(new Pose2d(new Vector2d(38, -48), Math.toRadians(-30)))
                                .strafeToSplineHeading(new Vector2d(50, -42), Math.toRadians(45))
                                .build(),
                        robot.intake.wristMoveAction(0.79),
                        new ParallelAction(
                                robot.intake.spinnerAction(1),
                                robot.intake.armUpAction(28),
                                robot.intake.checkForSample("blue", 8)
                        ),
                        new SleepAction(0.7),
                        new ParallelAction(
                                robot.intake.spinnerAction(0),
                                robot.intake.armDownAction(10),
                                robot.intake.wristMoveAction(0.5),
                                robot.drive.actionBuilder(new Pose2d(new Vector2d(44, -42), Math.toRadians(45)))
                                        .strafeToSplineHeading(new Vector2d(38, -48), Math.toRadians(-30))
                                        .build()
                        ),
                        robot.intake.spinnerAction(-1),
                        new SleepAction(1),
                        robot.intake.spinnerAction(0),
                        robot.intake.armDownAction(1)
                        ,
                        robot.drive.actionBuilder(new Pose2d(new Vector2d(38, -48), Math.toRadians(-30)))
                                .turn(Math.toRadians(120))
                                .strafeTo(parkPos)
                                .build(),
                        robot.intake.armUpAction(Intake.ARM_MAX_HORIZONTAL),
                        robot.intake.wristMoveAction(0)
                )
        ));

    }
}
