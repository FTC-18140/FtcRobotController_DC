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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeClaw;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot2024;
import org.firstinspires.ftc.teamcode.Robot.ThunderBot_Claw_2024;

@Config
@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ThunderBot_Claw_2024 robot = new ThunderBot_Claw_2024();
        robot.init(hardwareMap,telemetry, 0);
        Pose2d startPos = new Pose2d(-15,-60,Math.toRadians(90));
        robot.drive.pose = startPos;

        waitForStart();
        robot.intake.start();
        Actions.runBlocking(
                new ParallelAction(
                new SequentialAction(
                        robot.intake.clawAction(IntakeClaw.CLAW_OPEN),
                        robot.intake.armUpAction(37),
                        robot.intake.wristMoveAction(IntakeClaw.WRIST_MAX),
                        new ParallelAction(
                                robot.alignToColorAction(2)
                        ),
                        robot.intake.clawAction(IntakeClaw.CLAW_CLOSE),
                        new SleepAction(0.9),
                        robot.intake.presetAction(IntakeClaw.Positions.READY_TO_INTAKE)
                ),
                robot.intake.updateAction()
            )
        );

    }
}
